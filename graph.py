"""Plot motor speed and heading data collected from strikerforward.py."""

import ast
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple

import matplotlib.pyplot as plt

DEFAULT_SAMPLE_INTERVAL_MS = 10.0
HEADING_DRIFT_THRESHOLD_DEG = 10.0
MOTOR_ORDER = ("E", "A", "C", "D")
MOTOR_COLORS = {
    "E": "tab:blue",
    "A": "tab:orange",
    "C": "tab:green",
    "D": "tab:red",
}

# Paste your run data dictionary into PASTED_RUN_DATA to bypass all interactive input.
# Example:
# PASTED_RUN_DATA = {
#     "speed_samples": {
#         "E": [100, 110, 105],
#         "A": [98, 107, 104],
#         "C": [101, 109, 103],
#         "D": [99, 108, 106],
#     },
#     "sample_interval_ms": 10,
#     "reference_speed": 105,
#     "heading_samples": [0, 1, 2],
# }
PASTED_RUN_DATA = None  # Set to a dict to skip input.


def _safe_literal_eval(raw: str) -> Any:
    """Attempt to parse a Python literal, allowing for prefixed text."""

    literal_text = raw.strip()

    try:
        return ast.literal_eval(literal_text)
    except (SyntaxError, ValueError) as exc:
        start_index = literal_text.find("{")
        end_index = literal_text.rfind("}")
        if start_index != -1 and end_index != -1 and end_index > start_index:
            snippet = literal_text[start_index : end_index + 1]
            try:
                return ast.literal_eval(snippet)
            except (SyntaxError, ValueError) as inner_exc:  # pragma: no cover - helper
                raise ValueError("Input could not be parsed as a Python literal.") from inner_exc
        raise ValueError("Input could not be parsed as a Python literal.") from exc


def _normalize_sample_map(raw_map: Mapping[Any, Iterable]) -> Dict[str, List[float]]:
    """Validate and convert a mapping of motor labels to numeric samples."""

    processed: Dict[str, List[float]] = {}

    for motor, values in raw_map.items():
        if not isinstance(values, Iterable) or isinstance(values, (str, bytes)):
            raise ValueError(
                "Each dictionary value must be a list or tuple of numeric samples."
            )

        processed[str(motor)] = [float(value) for value in values]

    return processed


def _parse_run_data(
    raw: str,
) -> Tuple[Dict[str, List[float]], Optional[float], Optional[float], Optional[List[float]]]:
    """Parse the CLI input into samples and optional metadata."""

    try:
        data = _safe_literal_eval(raw)
    except ValueError as exc:  # pragma: no cover - interactive message
        raise ValueError("Input could not be parsed as a Python literal.") from exc

    if not isinstance(data, Mapping):
        raise ValueError("Input must be a dictionary of samples or run metadata.")

    if "speed_samples" in data:
        speed_section = data["speed_samples"]
        if not isinstance(speed_section, Mapping):
            raise ValueError("`speed_samples` must be a mapping of motor labels to lists.")

        samples = _normalize_sample_map(speed_section)

        sample_interval_ms: Optional[float]
        if "sample_interval_ms" in data:
            sample_interval_ms = float(data["sample_interval_ms"])
        else:
            sample_interval_ms = None

        reference_speed: Optional[float]
        if "reference_speed" in data:
            reference_speed = float(data["reference_speed"])
        else:
            reference_speed = None

        heading_samples: Optional[List[float]] = None
        if "heading_samples" in data:
            heading_section = data["heading_samples"]
            if not isinstance(heading_section, Iterable) or isinstance(
                heading_section, (str, bytes)
            ):
                raise ValueError("`heading_samples` must be a list of numeric values.")
            heading_samples = [float(value) for value in heading_section]

        return samples, sample_interval_ms, reference_speed, heading_samples

    samples = _normalize_sample_map(data)
    return samples, None, None, None


def main() -> None:
    """Collect user input and plot the provided speed samples."""
    # Fast path: user pasted dictionary above.
    if PASTED_RUN_DATA is not None:
        try:
            raw_samples = str(PASTED_RUN_DATA)
            samples, provided_interval, reference_speed, heading_samples = _parse_run_data(
                raw_samples
            )
        except ValueError as error:  # pragma: no cover - interactive message
            print(error)
            return

        sample_interval_ms = (
            provided_interval if provided_interval is not None else DEFAULT_SAMPLE_INTERVAL_MS
        )
        # No interactive overrides; proceed directly to plotting.

        ordered_samples: List[List[float]] = []
        for motor in MOTOR_ORDER:
            if motor in samples:
                ordered_samples.append(samples[motor])
        if not ordered_samples:
            print("No valid motor data found in the provided input.")
            return
        sample_lengths = {len(values) for values in ordered_samples}
        if len(sample_lengths) != 1:
            print("All motors must have the same number of samples.")
            return
        sample_count = sample_lengths.pop()
        if sample_count == 0:
            print("No samples found for the provided motors.")
            return
        times = [(index * sample_interval_ms) / 1000.0 for index in range(sample_count)]
        plt.figure()
        for motor in MOTOR_ORDER:
            if motor not in samples:
                continue
            plt.plot(times, samples[motor], label=f"Motor {motor}", color=MOTOR_COLORS.get(motor))
        if reference_speed is not None:
            plt.axhline(
                reference_speed,
                color="black",
                linestyle="--",
                linewidth=1,
                label="Reference speed",
            )
        drift_index: Optional[int] = None
        if heading_samples:
            compare_count = min(len(heading_samples), sample_count)
            baseline_heading = heading_samples[0]
            for index in range(compare_count):
                if abs(heading_samples[index] - baseline_heading) > HEADING_DRIFT_THRESHOLD_DEG:
                    drift_index = index
                    break
            if drift_index is not None:
                drift_time = times[min(drift_index, len(times) - 1)]
                plt.axvline(
                    drift_time,
                    color="black",
                    linestyle=":",
                    linewidth=1,
                    label="Heading drift threshold",
                )
            elif len(heading_samples) < sample_count:
                print(
                    "Heading samples shorter than speed samples; unable to mark drift threshold."
                )
            else:
                print("Heading never exceeded the drift threshold during the sampled window.")
        elif heading_samples == []:
            print("No heading data provided; skipping drift marker.")
        plt.title("Motor speed over time")
        plt.xlabel("Time (s)")
        plt.ylabel("Speed (deg/s)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()
        return

    # Original interactive path (unchanged below except for being inside else branch).
    raw_samples = input(
        "Paste the run data dictionary produced by strikerforward.py and press Enter:\n"
    )

    try:
        samples, provided_interval, reference_speed, heading_samples = _parse_run_data(
            raw_samples
        )
    except ValueError as error:  # pragma: no cover - interactive message
        print(error)
        return

    interval_default = (
        provided_interval if provided_interval is not None else DEFAULT_SAMPLE_INTERVAL_MS
    )
    sample_interval_text = input(
        (
            "Enter the sampling interval in milliseconds "
            f"[default: {interval_default}]: "
        )
    ).strip()

    if sample_interval_text:
        try:
            sample_interval_ms = float(sample_interval_text)
        except ValueError:  # pragma: no cover - interactive message
            print("Sampling interval must be a number.")
            return
    else:
        sample_interval_ms = interval_default

    if reference_speed is not None:
        reference_prompt = (
            "Enter the reference speed (deg/s) [default: {}]: ".format(reference_speed)
        )
    else:
        reference_prompt = "Enter the reference speed (deg/s) [optional]: "
    reference_text = input(reference_prompt).strip()
    if reference_text:
        try:
            reference_speed = float(reference_text)
        except ValueError:  # pragma: no cover - interactive message
            print("Reference speed must be a number.")
            return

    if heading_samples is None:
        heading_text = input(
            "Enter heading samples as a Python list (optional, used for drift marker): "
        ).strip()
        if heading_text:
            try:
                parsed_heading = _safe_literal_eval(heading_text)
            except ValueError:  # pragma: no cover - interactive message
                print("Heading samples could not be parsed as a Python list.")
                return

            if not isinstance(parsed_heading, Iterable) or isinstance(
                parsed_heading, (str, bytes)
            ):
                print("Heading samples must be provided as a list of numbers.")
                return

            heading_samples = [float(value) for value in parsed_heading]

    ordered_samples: List[List[float]] = []
    for motor in MOTOR_ORDER:
        if motor in samples:
            ordered_samples.append(samples[motor])

    if not ordered_samples:
        print("No valid motor data found in the provided input.")
        return

    sample_lengths = {len(values) for values in ordered_samples}
    if len(sample_lengths) != 1:
        print("All motors must have the same number of samples.")
        return

    sample_count = sample_lengths.pop()
    if sample_count == 0:
        print("No samples found for the provided motors.")
        return
    times = [(index * sample_interval_ms) / 1000.0 for index in range(sample_count)]

    plt.figure()
    for motor in MOTOR_ORDER:
        if motor not in samples:
            continue

        plt.plot(times, samples[motor], label=f"Motor {motor}", color=MOTOR_COLORS.get(motor))

    if reference_speed is not None:
        plt.axhline(
            reference_speed,
            color="black",
            linestyle="--",
            linewidth=1,
            label="Reference speed",
        )

    drift_index: Optional[int] = None
    if heading_samples:
        compare_count = min(len(heading_samples), sample_count)
        baseline_heading = heading_samples[0]
        for index in range(compare_count):
            if abs(heading_samples[index] - baseline_heading) > HEADING_DRIFT_THRESHOLD_DEG:
                drift_index = index
                break

        if drift_index is not None:
            drift_time = times[min(drift_index, len(times) - 1)]
            plt.axvline(
                drift_time,
                color="black",
                linestyle=":",
                linewidth=1,
                label="Heading drift threshold",
            )
        elif len(heading_samples) < sample_count:
            print(
                "Heading samples shorter than speed samples; unable to mark drift threshold."
            )
        else:
            print(
                "Heading never exceeded the drift threshold during the sampled window."
            )
    elif heading_samples == []:
        print("No heading data provided; skipping drift marker.")

    plt.title("Motor speed over time")
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (deg/s)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    main()
