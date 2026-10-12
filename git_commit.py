#!/usr/bin/env python3
"""
Компактный инструмент коммитов и синхронизации с Git для проекта 2-ННК.
Подавляет шум сотен служебных файлов OpenMC и предупреждений Windows CRLF,
экономя до 95% токенов при взаимодействии с Git.
"""
import sys
import subprocess
import argparse
import re


def run_cmd(cmd, input_text=None):
    """Выполняет команду git и возвращает stdout, stderr и код возврата."""
    res = subprocess.run(
        cmd,
        input=input_text,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        shell=False
    )
    return res.stdout.strip(), res.stderr.strip(), res.returncode


def get_compact_status():
    """Формирует ультра-компактный статус репозитория (1-3 строки)."""
    out, _, code = run_cmd(["git", "status", "--porcelain"])
    if code != 0 or not out:
        return "ℹ️ Рабочее дерево чистое, изменений нет."

    lines = out.splitlines()
    code_files = []
    data_files = []
    openmc_models = set()
    other_count = 0

    for line in lines:
        parts = line.strip().split(maxsplit=1)
        if len(parts) < 2:
            continue
        status, filepath = parts[0], parts[1]
        
        # Группируем файлы OpenMC
        if "openmc_cases/" in filepath or "openmc_cases\\" in filepath:
            m = re.search(r"openmc_cases[/\\]([^/\\]+)", filepath)
            if m:
                openmc_models.add(m.group(1))
            else:
                other_count += 1
        elif filepath.endswith((".py", ".bat", ".md", ".sh", ".json")):
            code_files.append(filepath)
        elif filepath.endswith((".csv", ".csv.tmp")):
            data_files.append(filepath)
        else:
            other_count += 1

    summary_parts = []
    if code_files:
        summary_parts.append(f"Код/конфиг ({len(code_files)}): {', '.join(code_files[:6])}{'...' if len(code_files) > 6 else ''}")
    if data_files:
        summary_parts.append(f"Базы данных ({len(data_files)}): {', '.join(data_files)}")
    if openmc_models:
        summary_parts.append(f"Модели OpenMC ({len(openmc_models)} кейсов, ~{len(lines) - len(code_files) - len(data_files)} файлов): {', '.join(sorted(openmc_models)[:4])}{'...' if len(openmc_models) > 4 else ''}")

    return "\n".join(summary_parts) if summary_parts else f"Изменено файлов: {len(lines)}"


def do_commit(message, desc="", do_push=True):
    """Индексирует, коммитит и отправляет изменения на GitHub с подавлением шума."""
    # 1. Проверяем наличие незакоммиченных изменений
    out, _, _ = run_cmd(["git", "status", "--porcelain"])
    if not out:
        print("ℹ️ Рабочее дерево чистое (новых файлов нет).")
        if do_push:
            print("🚀 Проверка и отправка накопленных коммитов на GitHub (git push)...")
            p_out, p_err, p_code = run_cmd(["git", "push"])
            if p_code == 0:
                print("✅ Все коммиты успешно отправлены на GitHub (origin/main).")
            else:
                print(f"❌ Ошибка при git push:\n{p_err or p_out}")
                return p_code
        return 0

    # 2. Индексируем всё (git add -A) с подавлением CRLF предупреждений
    _, _, code = run_cmd(["git", "add", "-A"])
    if code != 0:
        print("❌ Ошибка при выполнении git add -A")
        return 1

    # 3. Формируем стандартизированное сообщение
    commit_body_parts = [message.strip()]
    if desc.strip():
        commit_body_parts.append(f"\n{desc.strip()}")
    commit_body_parts.append(
        "\nGenerated with [Devin](https://devin.ai)\n\n"
        "Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>"
    )
    full_msg = "\n".join(commit_body_parts)

    # 4. Выполняем коммит
    res_out, res_err, res_code = run_cmd(["git", "commit", "-F", "-"], input_text=full_msg)
    if res_code != 0:
        print(f"❌ Ошибка коммита:\n{res_err or res_out}")
        return res_code

    # 5. Извлекаем краткую статистику коммита (1 строка)
    hash_out, _, _ = run_cmd(["git", "log", "-1", "--oneline"])
    stat_out, _, _ = run_cmd(["git", "diff", "--shortstat", "HEAD~1", "HEAD"])

    print(f"✅ Локально закоммичено: {hash_out}")
    if stat_out:
        print(f"📊 Статистика: {stat_out}")

    # 6. Отправка на GitHub по умолчанию (если не указан --no-push)
    if do_push:
        print("🚀 Отправка на GitHub (git push)...")
        p_out, p_err, p_code = run_cmd(["git", "push"])
        if p_code == 0:
            print("✅ Успешно синхронизировано с GitHub (origin/main)!")
        else:
            print(f"❌ Ошибка при git push:\n{p_err or p_out}")
            return p_code

    return 0


def main():
    parser = argparse.ArgumentParser(description="Компактный Git-коммитер для проекта 2-ННК (коммит + push).")
    parser.add_argument("message", nargs="?", default="", help="Заголовок коммита")
    parser.add_argument("-d", "--desc", default="", help="Подробное описание коммита")
    parser.add_argument("-s", "--status", action="store_true", help="Показать краткий статус (без коммита)")
    parser.add_argument("--push", action="store_true", help="Принудительно отправить на GitHub")
    parser.add_argument("--no-push", action="store_true", help="Только локальный коммит (без отправки на GitHub)")

    args = parser.parse_args()

    if args.status or (not args.message and not args.push):
        print("📋 Краткий статус репозитория:")
        print(get_compact_status())
        if not args.message:
            print("\nПодсказка: python git_commit.py \"Заголовок коммита\" [-d \"Описание\"]")
        return 0

    do_push = not args.no_push
    msg = args.message or "Sync commits with remote repository"
    return do_commit(msg, desc=args.desc, do_push=do_push)


if __name__ == "__main__":
    sys.exit(main())
