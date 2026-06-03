#!/usr/bin/env python3

from pathlib import Path


def replace_setting(text, name, value):
    lines = []
    replaced = False
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith(f"{name} "):
            comment = ""
            if "#" in line:
                comment = " " + line[line.index("#") :]
            lines.append(f"{name} = {value}{comment}")
            replaced = True
        else:
            lines.append(line)
    if not replaced:
        lines.append(f"{name} = {value}")
    return "\n".join(lines) + "\n"


def main():
    repo_root = Path(__file__).resolve().parents[1]
    config_path = repo_root / "external" / "unitree_mujoco" / "simulate_python" / "config.py"

    text = config_path.read_text(encoding="utf-8")
    text = replace_setting(text, "ROBOT", '"h1"')
    text = replace_setting(
        text,
        "ROBOT_SCENE",
        '"../unitree_robots/" + ROBOT + "/scene.xml"',
    )
    text = replace_setting(text, "DOMAIN_ID", "1")
    text = replace_setting(text, "INTERFACE", '"lo"')
    text = replace_setting(text, "USE_JOYSTICK", "0")
    text = replace_setting(text, "PRINT_SCENE_INFORMATION", "True")
    text = replace_setting(text, "ENABLE_ELASTIC_BAND", "True")
    text = replace_setting(text, "SIMULATE_DT", "0.005")
    text = replace_setting(text, "VIEWER_DT", "0.02")

    config_path.write_text(text, encoding="utf-8")
    print(f"Configured {config_path} for H1 on loopback DDS domain 1.")


if __name__ == "__main__":
    main()
