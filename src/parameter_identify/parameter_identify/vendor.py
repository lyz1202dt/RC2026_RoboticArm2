from __future__ import annotations

import sys
import importlib.util
from pathlib import Path
from types import ModuleType


def add_vendored_figaroh() -> Path:
    """Put the vendored FIGAROH package at the front of sys.path."""
    figaroh_src = _find_vendored_figaroh_src()
    if not figaroh_src.exists():
        raise FileNotFoundError(
            f"Vendored FIGAROH not found at {figaroh_src}. "
            "Clone https://github.com/thanhndv212/figaroh-plus.git there first."
        )
    src_text = str(figaroh_src)
    if src_text not in sys.path:
        sys.path.insert(0, src_text)
    return figaroh_src


def _find_vendored_figaroh_src() -> Path:
    candidates = []
    for base in [Path(__file__).resolve(), Path.cwd().resolve()]:
        candidates.extend(parent / "third_party" / "figaroh-plus" / "src" for parent in [base, *base.parents])
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return Path(__file__).resolve().parents[3] / "third_party" / "figaroh-plus" / "src"


def load_vendored_figaroh_module(module_name: str, relative_path: str) -> ModuleType:
    """Load one FIGAROH source file without importing figaroh.__init__."""
    figaroh_src = add_vendored_figaroh()
    module_path = figaroh_src / relative_path
    if not module_path.exists():
        raise FileNotFoundError(f"Vendored FIGAROH module not found: {module_path}")
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load module spec for {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module
