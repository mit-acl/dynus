#!/usr/bin/env python3
"""
Generate LaTeX table from benchmark data for DYNUS paper

This script reads the CSV benchmark data and generates a properly formatted
LaTeX table with best/worst highlighting.

Usage:
    python3 generate_latex_table.py
"""

import pandas as pd
import numpy as np
from pathlib import Path

# Configuration
ROOT_PATH = Path("/home/kkondo/code/dynus_ws/src/dynus/benchmark_data")
OUTPUT_FILE = Path("/home/kkondo/paper_writing/DYNUS_v3/tables/standardized_benchmark.tex")
VE_OUTPUT_FILE = Path("/home/kkondo/paper_writing/DYNUS_v3/tables/ve_benchmark.tex")

# Data files to load
DATA_FILES = {
    # (mode, planner, N) -> filename
    # FASTER original (only first control point constrained)
    ("single", "faster_orig", 4): "single_thread/original_faster_4_benchmark.csv",
    ("single", "faster_orig", 5): "single_thread/original_faster_5_benchmark.csv",
    ("single", "faster_orig", 6): "single_thread/original_faster_6_benchmark.csv",
    # FASTER safe (all control points constrained)
    ("single", "faster_safe", 4): "single_thread/faster_4_benchmark.csv",
    ("single", "faster_safe", 5): "single_thread/faster_5_benchmark.csv",
    ("single", "faster_safe", 6): "single_thread/faster_6_benchmark.csv",
    # DYNUS single
    ("single", "dynus", 4): "single_thread/dynus_4_benchmark.csv",
    ("single", "dynus", 5): "single_thread/dynus_5_benchmark.csv",
    ("single", "dynus", 6): "single_thread/dynus_6_benchmark.csv",
    # DYNUS multi
    ("multi", "dynus", 4): "multi_thread/dynus_4_benchmark.csv",
    ("multi", "dynus", 5): "multi_thread/dynus_5_benchmark.csv",
    ("multi", "dynus", 6): "multi_thread/dynus_6_benchmark.csv",
}

# SUPER data: loaded from CSV if available, otherwise fall back to hardcoded values
SUPER_CSV_PATH = ROOT_PATH / "default" / "super_benchmark.csv"

SUPER_DATA_FALLBACK = {
    "Algorithm": "SUPER",
    "Thread": "multi",
    "N": "--",
    "success_rate": 95.1,
    "per_opt_ms": 1.7,
    "total_opt_ms": 1.7,
    "traj_time_s": 16.8,
    "path_length": 8.8,
    "jerk_smooth": 1.4,
    "sfc_viol": 0.0,
    "vel_viol": 100.0,
    "acc_viol": 0.0,
    "jerk_viol": 0.0,
}


def safe_mean(series):
    """Compute mean, handling NaN"""
    s = series.dropna()
    return float(s.mean()) if not s.empty else np.nan


def load_super_data():
    """Load SUPER benchmark data from CSV, falling back to hardcoded values."""
    if not SUPER_CSV_PATH.exists():
        print(f"  SUPER CSV not found at {SUPER_CSV_PATH}, using hardcoded fallback values")
        return SUPER_DATA_FALLBACK

    print(f"  Loading SUPER data from {SUPER_CSV_PATH}")
    df = pd.read_csv(SUPER_CSV_PATH)

    df["success"] = pd.to_numeric(df["success"], errors="coerce").fillna(0).astype(int)

    success_rate = safe_mean(df["success"]) * 100
    succ_df = df[df["success"] == 1]

    per_opt_ms = safe_mean(succ_df["per_opt_runtime_ms"])
    total_opt_ms = safe_mean(succ_df["total_opt_runtime_ms"])
    traj_time_s = safe_mean(succ_df["total_traj_time_sec"])
    path_length = safe_mean(succ_df["traj_length_m"])
    jerk_smooth = safe_mean(succ_df["jerk_smoothness_l1"])

    # Violation rates: count / total * 100 (consistent with dynamic/static benchmarks)
    if "violation_total_samples" in succ_df.columns and "v_violation_count" in succ_df.columns:
        total_samples = succ_df["violation_total_samples"].sum()
        if total_samples > 0:
            sfc_viol = succ_df["corridor_violation_count"].sum() / total_samples * 100.0
            vel_viol = succ_df["v_violation_count"].sum() / total_samples * 100.0
            acc_viol = succ_df["a_violation_count"].sum() / total_samples * 100.0
            jerk_viol = succ_df["j_violation_count"].sum() / total_samples * 100.0
        else:
            sfc_viol = vel_viol = acc_viol = jerk_viol = 0.0
    else:
        sfc_viol = safe_mean(succ_df["corridor_violated"]) * 100 if "corridor_violated" in succ_df.columns else 0.0
        vel_viol = safe_mean(succ_df["v_violated"]) * 100 if "v_violated" in succ_df.columns else 0.0
        acc_viol = safe_mean(succ_df["a_violated"]) * 100 if "a_violated" in succ_df.columns else 0.0
        jerk_viol = safe_mean(succ_df["j_violated"]) * 100 if "j_violated" in succ_df.columns else 0.0

    return {
        "Algorithm": "SUPER",
        "Thread": "multi",
        "N": "--",
        "success_rate": success_rate,
        "per_opt_ms": per_opt_ms,
        "total_opt_ms": total_opt_ms,
        "traj_time_s": traj_time_s,
        "path_length": path_length,
        "jerk_smooth": jerk_smooth,
        "sfc_viol": sfc_viol,
        "vel_viol": vel_viol,
        "acc_viol": acc_viol,
        "jerk_viol": jerk_viol,
    }


SUPER_DATA = load_super_data()


def load_and_process_data():
    """Load all CSV files and compute statistics"""
    rows = []

    for (mode, planner, N), filename in DATA_FILES.items():
        filepath = ROOT_PATH / filename

        if not filepath.exists():
            print(f"WARNING: Missing {filepath}")
            continue

        df = pd.read_csv(filepath)

        # Parse success column
        df["success"] = pd.to_numeric(df["success"], errors="coerce").fillna(0).astype(int)

        # Compute success rate (percentage)
        success_rate = safe_mean(df["success"]) * 100

        # Compute means over successful runs only
        succ_df = df[df["success"] == 1]

        per_opt_ms = safe_mean(succ_df["per_opt_runtime_ms"])
        total_opt_ms = safe_mean(succ_df["total_opt_runtime_ms"])
        traj_time_s = safe_mean(succ_df["total_traj_time_sec"])
        path_length = safe_mean(succ_df["traj_length_m"])
        jerk_smooth = safe_mean(succ_df["jerk_smoothness_l1"])

        # Violation rates: count / total * 100 (consistent with dynamic/static benchmarks)
        # New CSVs have per-sample counts; fall back to binary flags for old CSVs
        if "violation_total_samples" in succ_df.columns and "v_violation_count" in succ_df.columns:
            total_samples = succ_df["violation_total_samples"].sum()
            if total_samples > 0:
                sfc_viol = succ_df["corridor_violation_count"].sum() / total_samples * 100.0
                vel_viol = succ_df["v_violation_count"].sum() / total_samples * 100.0
                acc_viol = succ_df["a_violation_count"].sum() / total_samples * 100.0
                jerk_viol = succ_df["j_violation_count"].sum() / total_samples * 100.0
            else:
                sfc_viol = vel_viol = acc_viol = jerk_viol = 0.0
        else:
            # Legacy fallback: binary per-case flags
            sfc_viol = safe_mean(succ_df["corridor_violated"]) * 100 if "corridor_violated" in succ_df.columns else 0.0
            vel_viol = safe_mean(succ_df["v_violated"]) * 100 if "v_violated" in succ_df.columns else 0.0
            acc_viol = safe_mean(succ_df["a_violated"]) * 100 if "a_violated" in succ_df.columns else 0.0
            jerk_viol = safe_mean(succ_df["j_violated"]) * 100 if "j_violated" in succ_df.columns else 0.0

        # Determine algorithm name
        if planner == "faster_orig":
            alg_name = "FASTER"
        elif planner == "faster_safe":
            alg_name = "FASTER (safe)"
        else:  # dynus
            alg_name = "DYNUS"

        rows.append({
            "Algorithm": alg_name,
            "Thread": mode,
            "N": N,
            "success_rate": success_rate,
            "per_opt_ms": per_opt_ms,
            "total_opt_ms": total_opt_ms,
            "traj_time_s": traj_time_s,
            "path_length": path_length,
            "jerk_smooth": jerk_smooth,
            "sfc_viol": sfc_viol,
            "vel_viol": vel_viol,
            "acc_viol": acc_viol,
            "jerk_viol": jerk_viol,
        })

    df = pd.DataFrame(rows)

    # Add SUPER data as first row
    super_df = pd.DataFrame([SUPER_DATA])
    df = pd.concat([super_df, df], ignore_index=True)

    return df


def find_best_worst(df, column, higher_is_better=False):
    """Find best and worst values in a column"""
    valid = df[column].dropna()
    if valid.empty:
        return None, None

    if higher_is_better:
        best = valid.max()
        worst = valid.min()
    else:
        best = valid.min()
        worst = valid.max()

    return best, worst


def format_value(val, best, worst, precision=1):
    """Format value with LaTeX highlighting"""
    if pd.isna(val):
        return "-"

    formatted = f"{val:.{precision}f}"

    # Check if best or worst (with small tolerance)
    tol = 0.01
    if abs(val - best) < tol:
        return f"\\best{{{formatted}}}"
    elif abs(val - worst) < tol:
        return f"\\worst{{{formatted}}}"
    else:
        return formatted


def generate_dynus_rows_only(df):
    """Generate only DYNUS rows (not full table) for manual insertion"""

    columns_config = [
        ("success_rate", "$R^{\\mathrm{opt}}_{\\mathrm{succ}}$ [\\%]", True, 1),
        ("per_opt_ms", "$T^{\\mathrm{per}}_{\\mathrm{opt}}$ [ms]", False, 1),
        ("total_opt_ms", "$T^{\\mathrm{total}}_{\\mathrm{opt}}$ [ms]", False, 1),
        ("traj_time_s", "$T_{\\mathrm{trav}}$ [s]", False, 1),
        ("path_length", "$L_{\\mathrm{path}}$ [m]", False, 1),
        ("jerk_smooth", "$S_{\\mathrm{jerk}}$ [m/s$^{2}$]", False, 1),
        ("sfc_viol", "$\\rho_{\\mathrm{sfc}}$ [\\%]", False, 1),
        ("vel_viol", "$\\rho_{\\mathrm{vel}}$ [\\%]", False, 1),
        ("acc_viol", "$\\rho_{\\mathrm{acc}}$ [\\%]", False, 1),
        ("jerk_viol", "$\\rho_{\\mathrm{jerk}}$ [\\%]", False, 1),
    ]

    # Filter to DYNUS only
    df_dynus = df[df["Algorithm"] == "DYNUS"].copy()

    if df_dynus.empty:
        return "% No DYNUS data found"

    # Find best/worst across ALL data (not just DYNUS) for fair comparison
    best_worst = {}
    for col_name, _, higher_better, _ in columns_config:
        best, worst = find_best_worst(df, col_name, higher_better)
        best_worst[col_name] = (best, worst)

    latex = []
    latex.append("% ========== DYNUS ROWS ONLY (copy into main table) ==========")
    latex.append("% Replace existing DYNUS rows with these updated values")
    latex.append("")

    # Group by N
    for N_val in sorted(df_dynus["N"].unique()):
        df_n = df_dynus[df_dynus["N"] == N_val].copy()

        # Sort: multi-thread first, then single-thread
        df_n = df_n.sort_values("Thread", ascending=False)  # multi before single

        latex.append(f"% N = {int(N_val)}")

        for idx, row in df_n.iterrows():
            thread = row["Thread"]

            # Build row (no N column - assume it's handled by multirow in main table)
            row_str = f"      DYNUS & {thread} &"

            for col_name, _, _, precision in columns_config:
                val = row[col_name]
                best, worst = best_worst[col_name]
                formatted = format_value(val, best, worst, precision)
                row_str += f" & {formatted}"

            row_str += " \\\\"
            latex.append(row_str)

        latex.append("")

    return "\n".join(latex)


def generate_latex_table(df):
    """Generate FULL LaTeX table code (use only if table doesn't exist yet)"""

    # Define columns and their properties
    # (column_name, latex_header, higher_is_better, precision)
    columns_config = [
        ("success_rate", "$R^{\\mathrm{opt}}_{\\mathrm{succ}}$ [\\%]", True, 1),
        ("per_opt_ms", "$T^{\\mathrm{per}}_{\\mathrm{opt}}$ [ms]", False, 1),
        ("total_opt_ms", "$T^{\\mathrm{total}}_{\\mathrm{opt}}$ [ms]", False, 1),
        ("traj_time_s", "$T_{\\mathrm{trav}}$ [s]", False, 1),
        ("path_length", "$L_{\\mathrm{path}}$ [m]", False, 1),
        ("jerk_smooth", "$S_{\\mathrm{jerk}}$ [m/s$^{2}$]", False, 1),
        ("sfc_viol", "$\\rho_{\\mathrm{sfc}}$ [\\%]", False, 1),
        ("vel_viol", "$\\rho_{\\mathrm{vel}}$ [\\%]", False, 1),
        ("acc_viol", "$\\rho_{\\mathrm{acc}}$ [\\%]", False, 1),
        ("jerk_viol", "$\\rho_{\\mathrm{jerk}}$ [\\%]", False, 1),
    ]

    # Start building LaTeX
    latex = []
    latex.append("\\begin{table*}")
    latex.append("  \\caption{Local trajectory optimization benchmarking results (computation time, performance, and constraint violation).")
    latex.append("  We mark in \\best{green} the best value in each column and in \\worst{red} the worst value.}")
    latex.append("  \\label{tab:standardized_benchmark}")
    latex.append("  \\centering")
    latex.append("  \\renewcommand{\\arraystretch}{1.2}")
    latex.append("  \\resizebox{\\textwidth}{!}{")
    latex.append("    \\begin{tabular}{c c c c c c c c c c c c c}")
    latex.append("      \\toprule")

    # Header rows
    latex.append("      \\multirow{2}{*}[-0.4ex]{\\textbf{Algorithm}}")
    latex.append("      & \\multirow{2}{*}[-0.4ex]{\\textbf{Thread}}")
    latex.append("      & \\multirow{2}{*}[-0.4ex]{\\textbf{N}}")
    latex.append("      & \\multicolumn{1}{c}{\\textbf{Success}}")
    latex.append("      & \\multicolumn{2}{c}{\\textbf{Computation Time}}")
    latex.append("      & \\multicolumn{3}{c}{\\textbf{Performance}}")
    latex.append("      & \\multicolumn{4}{c}{\\textbf{Constraint Violation}}")
    latex.append("      \\\\")
    latex.append("      \\cmidrule(lr){4-4}")
    latex.append("      \\cmidrule(lr){5-6}")
    latex.append("      \\cmidrule(lr){7-9}")
    latex.append("      \\cmidrule(lr){10-13}")

    # Column headers
    header_line = "      &&&"
    for _, latex_header, _, _ in columns_config:
        header_line += f"\n      {latex_header}"
        if latex_header != columns_config[-1][1]:  # not last
            header_line += " &"
    header_line += "\n      \\\\"
    latex.append(header_line)
    latex.append("      \\midrule")

    # Find best/worst for each column
    best_worst = {}
    for col_name, _, higher_better, _ in columns_config:
        best, worst = find_best_worst(df, col_name, higher_better)
        best_worst[col_name] = (best, worst)

    # Handle SUPER separately (appears first, alone)
    df_super = df[df["Algorithm"] == "SUPER"].copy()
    df_rest = df[df["Algorithm"] != "SUPER"].copy()

    # Add SUPER row if it exists
    if not df_super.empty:
        row = df_super.iloc[0]
        alg = row["Algorithm"]
        thread = row["Thread"]
        n_cell = "--"

        # Build row - SUPER has combined per/total opt time
        row_str = f"      {alg} & {thread} & {n_cell}"

        # Success rate
        val = row["success_rate"]
        formatted = format_value(val, np.nan, np.nan, 1)  # No best/worst for SUPER alone
        row_str += f" & {formatted}"

        # Combined per_opt and total_opt (use multicolumn)
        val = row["per_opt_ms"]
        formatted = f"{val:.1f}"  # SUPER has best value
        row_str += f" & \\multicolumn{{2}}{{c}}{{\\best{{{formatted}}}}}"

        # Performance metrics
        for col_name in ["traj_time_s", "path_length", "jerk_smooth"]:
            val = row[col_name]
            best, worst = best_worst[col_name]
            formatted = format_value(val, best, worst, 1)
            row_str += f" & {formatted}"

        # Violation rates
        for col_name in ["sfc_viol", "vel_viol", "acc_viol", "jerk_viol"]:
            val = row[col_name]
            best, worst = best_worst[col_name]
            formatted = format_value(val, best, worst, 1)
            row_str += f" & {formatted}"

        row_str += " \\\\"
        latex.append(row_str)
        latex.append("")
        latex.append("      \\midrule")
        latex.append("")

    # Group by N and add data rows for other algorithms
    for N_val in sorted(df_rest["N"].unique()):
        df_n = df_rest[df_rest["N"] == N_val].copy()

        # Sort by: FASTER, FASTER (safe), DYNUS single, DYNUS multi
        def sort_key(row):
            if row["Algorithm"] == "FASTER":
                return (0, 0)
            elif row["Algorithm"] == "FASTER (safe)":
                return (0, 1)
            elif row["Algorithm"] == "DYNUS" and row["Thread"] == "single":
                return (1, 0)
            else:  # DYNUS multi
                return (1, 1)

        df_n["sort_key"] = df_n.apply(sort_key, axis=1)
        df_n = df_n.sort_values("sort_key").drop(columns=["sort_key"])

        first_in_group = True
        for _, row in df_n.iterrows():
            # Algorithm name
            alg = row["Algorithm"]

            # Thread mode
            thread = row["Thread"]

            # N (use multirow for first entry of each N)
            if first_in_group:
                n_cell = f"\\multirow{{{len(df_n)}}}{{*}}{{{int(N_val)}}}"
                first_in_group = False
            else:
                n_cell = ""

            # Build row
            row_str = f"      {alg} & {thread} & {n_cell}"

            for col_name, _, _, precision in columns_config:
                val = row[col_name]
                best, worst = best_worst[col_name]
                formatted = format_value(val, best, worst, precision)
                row_str += f" & {formatted}"

            row_str += " \\\\"
            latex.append(row_str)

        # Add midrule between N groups (except after last)
        numeric_N = [n for n in df_rest["N"].unique() if isinstance(n, (int, float))]
        if numeric_N and N_val != max(numeric_N):
            latex.append("")
            latex.append("      \\midrule")
            latex.append("")

    latex.append("      \\bottomrule")
    latex.append("    \\end{tabular}")
    latex.append("  }")
    latex.append("  \\vspace{-1.0em}")
    latex.append("\\end{table*}")

    return "\n".join(latex)


def load_ve_data():
    """Load variable elimination benchmark data.

    VE=yes rows come from the standardized benchmark (multi_thread/dynus_N_benchmark.csv)
    so that both tables share the same data.  VE=no rows come from ve_benchmark/.
    """
    # VE=yes: reuse DYNUS multi-threaded data from standardized benchmark
    ve_yes_files = {
        N: ROOT_PATH / f"multi_thread/dynus_{N}_benchmark.csv"
        for N in [4, 5, 6]
    }
    # VE=no: dedicated without-VE runs
    ve_no_files = {
        N: ROOT_PATH / f"ve_benchmark/dynus_{N}_without_ve_benchmark.csv"
        for N in [4, 5, 6]
    }

    # Build combined file list: (N, ve_flag, filepath)
    file_list = []
    for N, fp in ve_yes_files.items():
        if fp.exists():
            file_list.append((N, "yes", fp))
        else:
            # Fall back to ve_benchmark with_ve file if multi_thread doesn't exist
            fallback = ROOT_PATH / f"ve_benchmark/dynus_{N}_with_ve_benchmark.csv"
            if fallback.exists():
                file_list.append((N, "yes", fallback))
            else:
                print(f"WARNING: Missing VE=yes data for N={N}")
    for N, fp in ve_no_files.items():
        if fp.exists():
            file_list.append((N, "no", fp))
        else:
            print(f"WARNING: Missing VE=no data for N={N}")

    if not file_list:
        print("WARNING: No VE benchmark data found")
        return pd.DataFrame()

    rows = []

    for N, ve_flag, csv_file in file_list:
        df = pd.read_csv(csv_file)

        # Parse success column
        df["success"] = pd.to_numeric(df["success"], errors="coerce").fillna(0).astype(int)

        # Compute success rate (percentage)
        success_rate = safe_mean(df["success"]) * 100

        # Compute means over successful runs only
        succ_df = df[df["success"] == 1]

        if succ_df.empty:
            # No successful runs - use NaN
            per_opt_ms = np.nan
            total_opt_ms = np.nan
            traj_time_s = np.nan
            path_length = np.nan
            jerk_smooth = np.nan
            any_viol = np.nan
        else:
            per_opt_ms = safe_mean(succ_df["per_opt_runtime_ms"])
            total_opt_ms = safe_mean(succ_df["total_opt_runtime_ms"])
            traj_time_s = safe_mean(succ_df["total_traj_time_sec"])
            path_length = safe_mean(succ_df["traj_length_m"])
            jerk_smooth = safe_mean(succ_df["jerk_smoothness_l1"])

            # Combined violation rate: count / total * 100 (consistent with dynamic/static benchmarks)
            if "violation_total_samples" in succ_df.columns and "v_violation_count" in succ_df.columns:
                total_samples = succ_df["violation_total_samples"].sum()
                if total_samples > 0:
                    sfc_viol = succ_df["corridor_violation_count"].sum() / total_samples * 100.0
                    vel_viol = succ_df["v_violation_count"].sum() / total_samples * 100.0
                    acc_viol = succ_df["a_violation_count"].sum() / total_samples * 100.0
                    jerk_viol = succ_df["j_violation_count"].sum() / total_samples * 100.0
                else:
                    sfc_viol = vel_viol = acc_viol = jerk_viol = 0.0
            else:
                sfc_viol = safe_mean(succ_df["corridor_violated"]) * 100 if "corridor_violated" in succ_df.columns else 0.0
                vel_viol = safe_mean(succ_df["v_violated"]) * 100 if "v_violated" in succ_df.columns else 0.0
                acc_viol = safe_mean(succ_df["a_violated"]) * 100 if "a_violated" in succ_df.columns else 0.0
                jerk_viol = safe_mean(succ_df["j_violated"]) * 100 if "j_violated" in succ_df.columns else 0.0

            # Max of all violations
            any_viol = max(sfc_viol, vel_viol, acc_viol, jerk_viol)

        rows.append({
            "N": N,
            "VE": ve_flag,
            "success_rate": success_rate,
            "per_opt_ms": per_opt_ms,
            "total_opt_ms": total_opt_ms,
            "traj_time_s": traj_time_s,
            "path_length": path_length,
            "jerk_smooth": jerk_smooth,
            "any_viol": any_viol,
        })

    return pd.DataFrame(rows)


def generate_ve_latex_table(df):
    """Generate LaTeX table for VE benchmark comparison"""

    if df.empty:
        return "% VE benchmark data not available"

    latex = []
    latex.append("\\begin{table}")
    latex.append("  \\caption{Variable Elimination (VE) Benchmarking Results in Standardized Environment: We highlight the best and worst values for each $N$ in \\best{green} and \\worst{red}, respectively. \\todo{add description}}")
    latex.append("  \\label{tab:variable_elimination_benchmark}")
    latex.append("  \\centering")
    latex.append("  \\renewcommand{\\arraystretch}{1.2}")
    latex.append("  \\resizebox{\\columnwidth}{!}{")
    latex.append("    \\begin{tabular}{c c c c c c c c}")
    latex.append("      \\toprule")

    # Column headers (single row, no sub-labels)
    latex.append("      \\textbf{N}")
    latex.append("      & \\textbf{VE}")
    latex.append("      & $R^{\\mathrm{opt}}_{\\mathrm{succ}}$ [\\%]")
    latex.append("      & \\shortstack{$T^{\\mathrm{per}}_{\\mathrm{opt}}${[ms]}}")
    latex.append("      & \\shortstack{$T_{\\mathrm{trav}}${[s]}}")
    latex.append("      & \\shortstack{$L_{\\mathrm{path}}${[m]}}")
    latex.append("      & \\shortstack{$S_{\\mathrm{jerk}}${[m/s$^{2}$]}}")
    latex.append("      & \\shortstack{$\\rho_{\\mathrm{viol}}${[\\%]}}")
    latex.append("      \\\\")
    latex.append("")
    latex.append("      \\midrule")
    latex.append("")

    # Data rows - group by N
    for N_val in sorted(df["N"].unique()):
        df_n = df[df["N"] == N_val].sort_values("VE", ascending=False)  # yes before no

        # Find best/worst for this N
        def find_best_worst_for_n(col, higher_better=False):
            vals = df_n[col].dropna()
            if vals.empty:
                return None, None
            if higher_better:
                return vals.max(), vals.min()
            else:
                return vals.min(), vals.max()

        best_worst_n = {
            "success_rate": find_best_worst_for_n("success_rate", True),
            "per_opt_ms": find_best_worst_for_n("per_opt_ms", False),
            "traj_time_s": find_best_worst_for_n("traj_time_s", False),
            "path_length": find_best_worst_for_n("path_length", False),
            "jerk_smooth": find_best_worst_for_n("jerk_smooth", False),
            "any_viol": find_best_worst_for_n("any_viol", False),
        }

        first_row = True
        for _, row in df_n.iterrows():
            # N column (multirow for first entry)
            if first_row:
                n_cell = f"\\multirow{{2}}{{*}}{{{int(N_val)}}}"
                first_row = False
            else:
                n_cell = ""

            # VE column
            ve_cell = "\\YesGreen" if row["VE"] == "yes" else "\\NoRed"

            row_str = f"      {n_cell} & {ve_cell}"

            # Success rate - green if 100.0, black otherwise
            val = row["success_rate"]
            if pd.isna(val):
                formatted = "-"
            elif abs(val - 100.0) < 0.01:
                # 100.0 -> green (best)
                formatted = f"\\best{{{val:.1f}}}"
            else:
                # Not 100.0 -> black (no highlighting)
                formatted = f"{val:.1f}"
            row_str += f" & {formatted}"

            # Per opt time
            val = row["per_opt_ms"]
            best, worst = best_worst_n["per_opt_ms"]
            row_str += f" & {format_value(val, best, worst, 1)}"

            # Traj time
            val = row["traj_time_s"]
            best, worst = best_worst_n["traj_time_s"]
            row_str += f" & {format_value(val, best, worst, 1)}"

            # Path length
            val = row["path_length"]
            best, worst = best_worst_n["path_length"]
            row_str += f" & {format_value(val, best, worst, 1)}"

            # Jerk smoothness
            val = row["jerk_smooth"]
            best, worst = best_worst_n["jerk_smooth"]
            row_str += f" & {format_value(val, best, worst, 1)}"

            # Combined violation
            val = row["any_viol"]
            best, worst = best_worst_n["any_viol"]
            row_str += f" & {format_value(val, best, worst, 1)}"

            row_str += " \\\\"
            latex.append(row_str)

        # Add midrule between N groups (except after last)
        if N_val != df["N"].max():
            latex.append("")
            latex.append("      \\midrule")
            latex.append("")

    latex.append("      \\bottomrule")
    latex.append("    \\end{tabular}")
    latex.append("  }")
    latex.append("  \\vspace{-1.0em}")
    latex.append("\\end{table}")

    return "\n".join(latex)


def main():
    """Main function"""
    import sys
    skip_ve = "--no-ve" in sys.argv

    print("="*80)
    print("DYNUS LaTeX Table Generator")
    print("="*80)

    # ========== Generate Standardized Benchmark Table ==========
    print("\n[1/2] Generating Standardized Benchmark Table")
    print("-" * 80)

    # Load data
    print("Loading benchmark data...")
    df = load_and_process_data()

    if df.empty:
        print("ERROR: No data loaded. Check that CSV files exist.")
    else:
        print(f"Loaded {len(df)} data rows")
        print("\nData summary:")
        print(df[["Algorithm", "Thread", "N"]].to_string(index=False))

        # Generate full LaTeX table
        print("\nGenerating FULL LaTeX table...")
        latex_code = generate_latex_table(df)

        # Save full table
        OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
        OUTPUT_FILE.write_text(latex_code)

        print(f"\n✓ Full LaTeX table saved to: {OUTPUT_FILE}")
        print(f"  Include in paper: \\input{{{OUTPUT_FILE.name}}}")

        # Generate DYNUS-only rows
        print("\nGenerating DYNUS-only rows...")
        dynus_rows = generate_dynus_rows_only(df)

        # Save DYNUS-only rows
        dynus_only_file = OUTPUT_FILE.parent / "dynus_rows_only.tex"
        dynus_only_file.write_text(dynus_rows)

        print(f"\n✓ DYNUS-only rows saved to: {dynus_only_file}")
        print(f"\n{'='*80}")
        print("USAGE INSTRUCTIONS:")
        print("="*80)
        print("\nOption 1: Use full table (if starting fresh)")
        print(f"  \\input{{{OUTPUT_FILE.name}}}")
        print("\nOption 2: Update existing table (preserves other planners)")
        print(f"  1. Open your existing table file")
        print(f"  2. Find all lines containing 'DYNUS'")
        print(f"  3. Replace them with contents from: {dynus_only_file.name}")
        print(f"  4. Make sure multirow{{N}} values match your table structure")
        print("="*80)

    # ========== Generate VE Benchmark Table ==========
    ve_df = pd.DataFrame()
    if skip_ve:
        print("\n[2/2] Skipping VE Benchmark Table (--no-ve)")
    else:
        print("\n[2/2] Generating Variable Elimination Benchmark Table")
        print("-" * 80)

        # Load VE data
        print("Loading VE benchmark data...")
        ve_df = load_ve_data()

        if ve_df.empty:
            print("WARNING: No VE benchmark data found.")
            print("  Run: python3 run_benchmark_suite.py --ve-comparison")
        else:
            print(f"Loaded {len(ve_df)} VE data rows")
            print("\nVE data summary:")
            print(ve_df[["N", "VE"]].to_string(index=False))

            # Generate LaTeX
            print("\nGenerating VE LaTeX table...")
            ve_latex_code = generate_ve_latex_table(ve_df)

            # Save to file
            VE_OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
            VE_OUTPUT_FILE.write_text(ve_latex_code)

            print(f"\n✓ VE LaTeX table saved to: {VE_OUTPUT_FILE}")
            print(f"  Include in paper: \\input{{{VE_OUTPUT_FILE.name}}}")

    # ========== Summary ==========
    print("\n" + "="*80)
    print("GENERATION COMPLETE")
    print("="*80)
    print("\nGenerated files:")
    if not df.empty:
        print(f"  1. {OUTPUT_FILE}")
    if not ve_df.empty:
        print(f"  2. {VE_OUTPUT_FILE}")


if __name__ == "__main__":
    main()
