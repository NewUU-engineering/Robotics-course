    rules_provided: bool,
    diff_provided: bool,
) -> dict[str, Any]:
    verdict = str(report.get("verdict", "needs_manual_review")).lower()
    if verdict not in {"pass", "partial", "fail", "needs_manual_review"}:
        verdict = "needs_manual_review"

    findings: list[dict[str, Any]] = []
    raw_findings = report.get("findings", [])
    if isinstance(raw_findings, list):
        for finding in raw_findings:
            if not isinstance(finding, dict):
                continue
            status = str(finding.get("status", "needs_confirmation")).lower()
            if status not in {"confirmed", "candidate", "needs_confirmation"}:
                status = "needs_confirmation"
            confidence = str(finding.get("confidence", "low")).lower()
            if confidence not in {"high", "medium", "low"}:
                confidence = "low"
            basis = str(finding.get("basis", "unknown")).lower()
            if basis not in {"test", "documentation", "code_invariant", "inference", "unknown"}:
                basis = "unknown"
            severity = str(finding.get("severity", "INFO")).upper()
            if severity not in {"CRITICAL", "WARNING", "INFO"}:
                severity = "INFO"
            findings.append(
                {
                    "status": status,
                    "confidence": confidence,
                    "basis": basis,
                    "severity": severity,
                    "file": finding.get("file"),
                    "line": finding.get("line"),
                    "cell": finding.get("cell"),
                    "location": finding.get("location"),
                    "title": str(finding.get("title", "Без заголовка")),
                    "evidence": str(finding.get("evidence", "")),
                    "description": str(finding.get("description", "")),
                    "recommendation": str(finding.get("recommendation", "")),
                }
            )

    raw_questions = report.get("questions", [])
    questions = [str(question) for question in raw_questions] if isinstance(raw_questions, list) else []

    confirmed_count = sum(item["status"] == "confirmed" for item in findings)
    candidate_count = sum(item["status"] == "candidate" for item in findings)
    confirmation_count = sum(item["status"] == "needs_confirmation" for item in findings)
    needs_manual_review = bool(report.get("needs_manual_review", verdict == "needs_manual_review"))
    if candidate_count or confirmation_count:
        needs_manual_review = True
    if findings and not has_trusted_facts and confirmed_count == 0:
        verdict = "needs_manual_review"

    return {
        "summary": str(report.get("summary", "Итог не сформирован.")),
        "verdict": verdict,
        "findings": findings,
        "questions": questions,
        "needs_manual_review": needs_manual_review,
        "finding_summary": {
            "confirmed": confirmed_count,
            "candidate": candidate_count,
            "needs_confirmation": confirmation_count,
        },
        "reviewed_files": [item.relative_path for item in reviewed_files],
        "review_context": {
            "trusted_autograder_facts": has_trusted_facts,
            "project_rules": rules_provided,
            "diff": diff_provided,
        },
        "model": model,
        "generated_at": datetime.now(timezone.utc).isoformat(),
    }


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Локальный read-only ревьюер .md, .py и .ipynb через Gemini")
    parser.add_argument("path", help="Файл или папка для ревью")
    parser.add_argument("--output", default="review_report.json", help="Путь для JSON-отчёта")
    parser.add_argument("--facts", help="Необязательный JSON/текст результата детерминированного автотеста")
    parser.add_argument("--diff", help="Необязательный файл с Git diff")
    parser.add_argument("--rules", help="Необязательный файл с требованиями и известными нормальными особенностями проекта")
    parser.add_argument("--model", default=os.getenv("GEMINI_MODEL", "gemini-3.8-flash"))
    parser.add_argument("--max-file-bytes", type=int, default=60_000)
    parser.add_argument("--max-total-bytes", type=int, default=240_000)
    parser.add_argument("--dry-run", action="store_true", help="Только подготовить prompt, без вызова API")
    parser.add_argument("--prompt-output", default="review_prompt.txt")
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()

    try:
        root = Path(args.path).expanduser().resolve()
        files = load_source_files(root, args.max_file_bytes, args.max_total_bytes)
        facts = read_optional_text(args.facts, "Файл автотеста", args.max_total_bytes)
        diff = read_optional_text(args.diff, "Файл diff", args.max_total_bytes)
        rules = read_optional_text(args.rules, "Файл проектных правил", args.max_total_bytes)
        prompt = build_prompt(files, facts, diff, rules)

        if args.dry_run:
            prompt_path = Path(args.prompt_output).expanduser().resolve()
            prompt_path.write_text(prompt, encoding="utf-8")
            print(f"Prompt сохранён: {prompt_path}")
            return 0

        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ReviewerError(
                "Не найден GEMINI_API_KEY. Сначала задайте его только в текущем окне PowerShell."
            )

        response = call_gemini(api_key, args.model, prompt)
        model_text = extract_model_text(response)
        report = normalize_report(
            parse_json_report(model_text),
            files,
            args.model,
            has_trusted_facts=bool(facts),
            rules_provided=bool(rules),
            diff_provided=bool(diff),
        )
        output_path = Path(args.output).expanduser().resolve()
        output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"Отчёт сохранён: {output_path}")
        print(f"Вердикт: {report['verdict']}")
