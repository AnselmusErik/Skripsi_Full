import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import os

# Configuration
REPORTS_DIR = Path("reports/ldr")
PLOTS_DIR = Path("plots")
METRICS = ["aborted", "dropped", "delivered", "delivery_prob"]

# Create plots directory if it doesn't exist
PLOTS_DIR.mkdir(parents=True, exist_ok=True)

def parse_message_stats_report(csv_path):
    """
    Parse MessageStatsReport.csv (key-value format) and extract metrics.
    Returns dict with metric names as keys and values as floats.
    """
    metrics = {}
    try:
        with open(csv_path, 'r') as f:
            for line in f:
                line = line.strip()
                if ':' in line:
                    key, value = line.split(':', 1)
                    key = key.strip()
                    value = value.strip()

                    if key in METRICS:
                        try:
                            metrics[key] = float(value)
                        except ValueError:
                            metrics[key] = np.nan
    except Exception as e:
        print(f"Error reading {csv_path}: {e}")

    return metrics

def load_all_reports():
    """
    Load all MessageStatsReport.csv files from all configuration folders.
    Returns dataframe with columns: [config, run, aborted, dropped, delivered, delivery_prob]
    """
    data = []

    # Iterate through all config folders (dt@5, dt@10, ..., epidemic)
    for config_dir in sorted(REPORTS_DIR.iterdir()):
        if not config_dir.is_dir():
            continue

        config_name = config_dir.name

        # Iterate through all run subdirectories (numbered 1, 2, 3, ...)
        for run_dir in sorted(config_dir.iterdir()):
            if not run_dir.is_dir():
                continue

            try:
                run_num = int(run_dir.name)
            except ValueError:
                continue

            # Find MessageStatsReport.csv file
            csv_files = list(run_dir.glob("*MessageStatsReport.csv"))
            if not csv_files:
                continue

            csv_path = csv_files[0]
            metrics = parse_message_stats_report(csv_path)

            if metrics:
                row = {"config": config_name, "run": run_num}
                row.update(metrics)
                data.append(row)

    df = pd.DataFrame(data)
    return df

def calculate_statistics(df):
    """
    Group by configuration and calculate min, avg, max for each metric.
    Returns dataframe with columns: [config, metric_name, min, avg, max]
    """
    stats_list = []

    for config in sorted(df['config'].unique()):
        config_data = df[df['config'] == config]

        for metric in METRICS:
            values = config_data[metric].dropna()

            if len(values) > 0:
                stats_list.append({
                    'config': config,
                    'metric': metric,
                    'min': values.min(),
                    'avg': values.mean(),
                    'max': values.max()
                })

    stats_df = pd.DataFrame(stats_list)
    return stats_df

def get_colors(n_configs):
    """Get n_configs colors from gist_rainbow colormap"""
    cmap = plt.get_cmap('gist_rainbow')
    return [cmap(i / max(1, n_configs - 1)) for i in range(n_configs)]

def plot_line_chart(df, metric):
    """Create line plot with averaged values per config"""
    fig, ax = plt.subplots(figsize=(10, 6))

    configs = sorted(df[df['metric'] == metric]['config'].unique())
    colors = get_colors(len(configs))

    stats_df = df[df['metric'] == metric].set_index('config')

    for i, config in enumerate(configs):
        if config in stats_df.index:
            avg_value = stats_df.loc[config, 'avg']
            ax.plot([i], [avg_value], 'o-', color=colors[i], linewidth=2,
                   markersize=8, label=config)

    ax.set_xticks(range(len(configs)))
    ax.set_xticklabels(configs, rotation=45, ha='right')
    ax.set_ylabel('Value')
    ax.set_title(f'Line Plot - {metric.replace("_", " ").title()}')
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    output_path = PLOTS_DIR / f"01_line_{metric}.png"
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {output_path}")

def plot_bar_chart(df, metric):
    """Create bar plot with error bars (min-max) around average"""
    fig, ax = plt.subplots(figsize=(10, 6))

    configs = sorted(df[df['metric'] == metric]['config'].unique())
    colors = get_colors(len(configs))

    stats_df = df[df['metric'] == metric].set_index('config')

    x_pos = np.arange(len(configs))
    avgs = []
    errors_lower = []
    errors_upper = []

    for config in configs:
        if config in stats_df.index:
            avg = stats_df.loc[config, 'avg']
            min_val = stats_df.loc[config, 'min']
            max_val = stats_df.loc[config, 'max']

            avgs.append(avg)
            errors_lower.append(avg - min_val)
            errors_upper.append(max_val - avg)

    bars = ax.bar(x_pos, avgs, color=colors, edgecolor='black', linewidth=1.5)

    # Add error bars (min to max range)
    ax.errorbar(x_pos, avgs,
               yerr=[errors_lower, errors_upper],
               fmt='none', color='black', capsize=5, capthick=2,
               elinewidth=2, label='Min-Max Range')

    ax.set_xticks(x_pos)
    ax.set_xticklabels(configs, rotation=45, ha='right')
    ax.set_ylabel('Value')
    ax.set_title(f'Bar Plot - {metric.replace("_", " ").title()}')
    ax.grid(True, alpha=0.3, axis='y')
    ax.legend()
    fig.tight_layout()

    output_path = PLOTS_DIR / f"02_bar_{metric}.png"
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {output_path}")

def plot_line_dot_chart(df, metric):
    """Create line-dot plot with markers and error bars (min-max)"""
    fig, ax = plt.subplots(figsize=(10, 6))

    configs = sorted(df[df['metric'] == metric]['config'].unique())
    colors = get_colors(len(configs))

    stats_df = df[df['metric'] == metric].set_index('config')

    x_pos = np.arange(len(configs))
    avgs = []
    errors_lower = []
    errors_upper = []

    for config in configs:
        if config in stats_df.index:
            avg = stats_df.loc[config, 'avg']
            min_val = stats_df.loc[config, 'min']
            max_val = stats_df.loc[config, 'max']

            avgs.append(avg)
            errors_lower.append(avg - min_val)
            errors_upper.append(max_val - avg)

    # Plot line with markers
    ax.plot(x_pos, avgs, 'o-', color='gray', linewidth=2, markersize=0, alpha=0.5)

    # Plot colored dots and error bars
    for i, (config, color) in enumerate(zip(configs, colors)):
        ax.errorbar(i, avgs[i],
                   yerr=[[errors_lower[i]], [errors_upper[i]]],
                   fmt='o', color=color, markersize=10, capsize=5,
                   capthick=2, elinewidth=2, label=config)

    ax.set_xticks(x_pos)
    ax.set_xticklabels(configs, rotation=45, ha='right')
    ax.set_ylabel('Value')
    ax.set_title(f'Line-Dot Plot - {metric.replace("_", " ").title()}')
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    output_path = PLOTS_DIR / f"03_line_dot_{metric}.png"
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {output_path}")

def main():
    print("Loading reports...")
    df = load_all_reports()

    if df.empty:
        print("No data found!")
        return

    print(f"Loaded {len(df)} reports from {df['config'].nunique()} configurations")
    print(f"Configurations: {sorted(df['config'].unique())}")

    print("\nCalculating statistics...")
    stats_df = calculate_statistics(df)

    print("\nGenerating plots...")
    for metric in METRICS:
        print(f"\nProcessing metric: {metric}")
        plot_line_chart(stats_df, metric)
        plot_bar_chart(stats_df, metric)
        plot_line_dot_chart(stats_df, metric)

    print(f"\n✓ All plots saved to: {PLOTS_DIR.resolve()}")

if __name__ == "__main__":
    main()
