#!/usr/bin/env python3
"""Synchronize complete example files into the chapter that runs each example."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHAPTERS = ROOT / "chapters"
EXAMPLES = ROOT / "examples"
VIEWER_EXAMPLES = {
    "03_pd_control", "08_split_step", "32_damped_ik",
    "37_ekf_pendulum", "45_arm_reach", "46_biped_standing",
}
VIEW_MODELS = {"38_mjspec_build", "43_sensor_plugin", "44_vfs_model"}
EXAMPLE_RE = re.compile(r"cd examples/([0-9]{2}_[A-Za-z0-9_]+)")
BLOCK_RE = re.compile(
    r"\n?<!-- EMBEDDED_EXAMPLE_BEGIN: ([^ ]+) -->.*?"
    r"<!-- EMBEDDED_EXAMPLE_END: \1 -->\n?",
    re.DOTALL,
)
FENCE_RE = re.compile(r"^```[^\n]*\n.*?^```[ \t]*$", re.MULTILINE | re.DOTALL)
HEADING_RE = re.compile(r"^(#{1,6})\s+.+$", re.MULTILINE)


def fenced(language: str, content: str) -> str:
    return f"```{language}\n{content.rstrip()}\n```"


def source_block(name: str) -> str:
    directory = EXAMPLES / name
    files = [
        ("model.xml", "xml", "模型文件：`model.xml`"),
        ("view.xml", "xml", "可视化模型：`view.xml`"),
        ("g1.xml", "xml", "机器人模型：`g1.xml`"),
        ("main.cc", "cpp", "程序源码：`main.cc`"),
        ("CMakeLists.txt", "cmake", "构建文件：`CMakeLists.txt`"),
    ]
    sections = []
    for filename, language, title in files:
        path = directory / filename
        if path.exists():
            sections.append(f"#### {title}\n\n{fenced(language, path.read_text())}")

    body = "\n\n".join(sections)
    if name in VIEWER_EXAMPLES:
        command = f"./build/demo model.xml --view"
        note = (
            "窗口显示的是示例算法正在修改和推进的同一个 `mjData`。"
            "源码有意不封装 viewer：先用 GLFW 创建 OpenGL context，再初始化 "
            "`mjvScene/mjrContext`，用 `mjv_updateScene`读取算法使用的 `mjData`，"
            "再调用 `mjr_render` 和交换缓冲区，最后按创建的逆序释放资源。"
        )
    else:
        model = "view.xml" if name in VIEW_MODELS else "model.xml"
        command = f"../../mujoco-3.11.0/bin/simulate {model}"
        note = "该命令使用发布包自带的官方 `simulate` 界面，可暂停、单步、施加扰动并开启接触等可视化标志。"
    visual = (
        "### 可视化运行与效果\n\n"
        f"```bash\n{command}\n```\n\n{note}\n\n"
        f"![{name} 实验运行效果](../assets/experiments/{name}.png)\n\n"
        f"*{name} 的真实 MuJoCo 原生渲染结果。*"
    )
    return (
        f"<!-- EMBEDDED_EXAMPLE_BEGIN: {name} -->\n"
        f"{visual}\n\n"
        f"### 实验完整源码\n\n"
        f"以下文件与 `{directory.relative_to(ROOT)}/` 中可直接编译的版本一致。\n\n"
        f"{body}\n"
        f"<!-- EMBEDDED_EXAMPLE_END: {name} -->"
    )


def insertion_point(text: str, command_start: int) -> int:
    fence = next((item for item in FENCE_RE.finditer(text) if item.start() <= command_start <= item.end()), None)
    if fence is None:
        raise ValueError("example command is not inside a fenced code block")

    headings = [item for item in HEADING_RE.finditer(text) if item.start() < fence.start()]
    if not headings:
        raise ValueError("example command has no containing section")
    level = len(headings[-1].group(1))

    for heading in HEADING_RE.finditer(text, fence.end()):
        if len(heading.group(1)) <= level:
            return heading.start()
    return len(text)


def strip_blocks(text: str) -> str:
    return BLOCK_RE.sub("\n", text)


def expected_chapter_text(path: Path) -> tuple[str, list[str]]:
    text = strip_blocks(path.read_text())
    names = list(dict.fromkeys(EXAMPLE_RE.findall(text)))
    for name in reversed(names):
        command = f"cd examples/{name}"
        command_start = text.index(command)
        point = insertion_point(text, command_start)
        block = source_block(name)
        text = text[:point].rstrip() + "\n\n" + block + "\n\n" + text[point:].lstrip()
    return text.rstrip() + "\n", names


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="report stale chapter embeddings")
    args = parser.parse_args()

    seen: dict[str, Path] = {}
    stale = []
    for path in sorted(CHAPTERS.glob("*.md")):
        expected, names = expected_chapter_text(path)
        for name in names:
            if name in seen:
                raise SystemExit(f"example {name} is run by both {seen[name]} and {path}")
            seen[name] = path
        if path.read_text() != expected:
            stale.append(path)
            if not args.check:
                path.write_text(expected)

    directories = {path.name for path in EXAMPLES.iterdir() if path.is_dir()}
    missing = sorted(directories - set(seen))
    unknown = sorted(set(seen) - directories)
    if missing or unknown:
        if missing:
            print("examples missing from chapters:", ", ".join(missing))
        if unknown:
            print("chapter commands with no example directory:", ", ".join(unknown))
        return 1
    if args.check and stale:
        print("stale embedded sources:")
        for path in stale:
            print(path.relative_to(ROOT))
        return 1

    action = "checked" if args.check else "synchronized"
    print(f"{action} {len(seen)} examples in {len(set(seen.values()))} chapters")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
