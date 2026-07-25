from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_branch_declares_jazzy_only():
    assert (ROOT / ".rosdistro").read_text().strip() == "jazzy"
    assert not (ROOT / "ros_ws/humble").exists()


def test_generated_products_are_ignored():
    ignored = (ROOT / ".gitignore").read_text()
    for pattern in (
        "__pycache__/",
        ".pytest_cache/",
        "ros_ws/build/",
        "ros_ws/install/",
        "ros_ws/log/",
        "*.hdf5",
        "*.db3",
    ):
        assert pattern in ignored
