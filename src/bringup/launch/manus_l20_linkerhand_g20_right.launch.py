from pathlib import Path
import importlib.util


def _load_generate_manus_l20_launch():
    helper_path = Path(__file__).resolve().parent / "manus_l20_common" / "pipeline.py"
    spec = importlib.util.spec_from_file_location("manus_l20_common_pipeline", helper_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load launch helper: {helper_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_manus_l20_launch


def generate_launch_description():
    generate_manus_l20_launch = _load_generate_manus_l20_launch()
    return generate_manus_l20_launch(
        hand_type="right",
        retarget_node_name="manus_l20_retarget_right",
    )
