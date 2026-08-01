"""docs/cli.md must describe the parser that actually ships.

Walking argparse rather than reading a hand-kept list means a new subcommand
or option fails here the moment it is added without being documented.
"""

import argparse
from pathlib import Path
import re

import pytest

from robot_control.cli import _parser

DOCUMENT = Path(__file__).parents[1] / "docs/cli.md"

# Options every subparser inherits or that argparse supplies; documenting each
# one per command would be noise.
UNIVERSAL = {"-h", "--help"}


def _walk(parser: argparse.ArgumentParser, path: tuple[str, ...] = ()):
    """Yield (command path, option strings) for the parser and every subparser."""
    options: set[str] = set()
    subparsers: list[tuple[str, argparse.ArgumentParser]] = []
    for action in parser._actions:
        if isinstance(action, argparse._SubParsersAction):
            subparsers += list(action.choices.items())
        else:
            options.update(action.option_strings)
    yield path, options - UNIVERSAL
    for name, child in subparsers:
        yield from _walk(child, path + (name,))


@pytest.fixture(scope="module")
def document() -> str:
    assert DOCUMENT.is_file(), f"{DOCUMENT} is a required deliverable"
    return DOCUMENT.read_text()


@pytest.fixture(scope="module")
def commands():
    return [(path, options) for path, options in _walk(_parser()) if path]


def test_every_subcommand_is_documented(document, commands):
    missing = [
        " ".join(path)
        for path, _options in commands
        if f"robotctl {' '.join(path)}" not in document
    ]

    assert not missing, f"undocumented commands: {missing}"


def test_every_option_is_documented(document, commands):
    missing = []
    for path, options in commands:
        for option in options:
            if option not in document:
                missing.append(f"{' '.join(path)} {option}")

    assert not missing, f"undocumented options: {sorted(missing)}"


def test_document_records_the_exit_code_convention(document):
    for code in ("0", "2", "3"):
        assert re.search(rf"^\|\s*`{code}`\s*\|", document, re.MULTILINE), code


def test_every_execute_example_carries_a_safety_note(document):
    """--execute must never appear in a block that does not say it publishes."""
    blocks = re.findall(r"```bash\n(.*?)```", document, re.DOTALL)
    executing = [block for block in blocks if "--execute" in block]

    assert executing, "the document shows no execute example at all"
    for block in executing:
        assert re.search(
            r"#.*(publish|move|send|real|command)", block, re.IGNORECASE
        ), f"execute example with no safety comment:\n{block}"


def test_document_warns_about_replacing_pythonpath(document):
    """Assigning PYTHONPATH instead of appending hides rclpy; it cost a debug cycle."""
    assert 'PYTHONPATH="src:.:$PYTHONPATH"' in document
