from io import BytesIO
import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import pickle
import os
import imageio.v2 as imageio
from tqdm import tqdm
from PIL import Image
from matplotlib import gridspec


def plot_response(t, u, y, u_obs=None, y_obs=None, set_point=None, identifier=None):
    print(f"Plotting response with identifier: {identifier}")
    if t is None:
        return

    # Remove NaN values
    t = np.array(t)
    u = np.array(u)
    y = np.array(y)
    if u_obs is not None:
        u_obs = np.array(u_obs)
    if y_obs is not None:
        y_obs = np.array(y_obs)

    mask = ~np.isnan(t) & ~np.isnan(u) & ~np.isnan(y)
    t = t[mask]
    u = u[mask]
    y = y[mask]
    if u_obs is not None:
        u_obs = u_obs[mask]
    if y_obs is not None:
        y_obs = y_obs[mask]

    plt.figure()
    plt.subplot(1, 2, 1)
    plt.plot(t, y, label="Output Real")
    plt.xlabel("Time (s)")
    plt.ylabel("Output")
    plt.title("Response")
    if set_point is not None:
        plt.axhline(y=set_point, color="r", linestyle="--", label="Set Point")
    plt.grid()
    plt.legend()

    plt.subplot(1, 2, 2)
    plt.plot(t, u, label="Control Signal Real")
    plt.xlabel("Time (s)")
    plt.ylabel("Input")
    plt.title("Control Signal")
    plt.grid()
    plt.legend()

    if u_obs is not None and y_obs is not None:
        plt.subplot(1, 2, 1)
        plt.plot(t, y_obs, label="Output Observer")
        plt.legend()

        plt.subplot(1, 2, 2)
        plt.plot(t, u_obs, label="Control Signal Observer")
        plt.legend()

    plt.show()


import matplotlib.pyplot as plt
import numpy as np
import os

def plot_all_comparative(
    t,
    u_step,
    y_step,
    u_pid,
    y_pid,
    u_smith,
    y_smith,
    u_obs=None,
    y_obs=None,
    set_point=None,
    identifier=None,
):
    """
    Plots comparative system responses: step, PID vs. Smith predictor,
    and optionally observer. Saves figures in 'identifier' folder if provided.
    """
    if t is None or len(t) == 0:
        print("No time vector provided, skipping plots.")
        return

    # Prepare output directory if identifier is given
    save_dir = None
    if identifier is not None:
        save_dir = os.path.join(os.getcwd(), str(identifier))
        os.makedirs(save_dir, exist_ok=True)

    # Convert to arrays and remove NaNs
    t = np.asarray(t)
    y_step = np.asarray(y_step)
    y_pid = np.asarray(y_pid)
    y_smith = np.asarray(y_smith)
    u_pid = np.asarray(u_pid)
    u_smith = np.asarray(u_smith)

    mask = (
        ~np.isnan(t)
        & ~np.isnan(y_step)
        & ~np.isnan(y_pid)
        & ~np.isnan(y_smith)
        & ~np.isnan(u_pid)
        & ~np.isnan(u_smith)
    )
    t = t[mask]
    y_step = y_step[mask]
    y_pid = y_pid[mask]
    y_smith = y_smith[mask]
    u_pid = u_pid[mask]
    u_smith = u_smith[mask]

    # Matplotlib style for academic quality
    plt.rcParams.update({
        'text.usetex': True,
        'font.family': 'serif',
        'font.serif': ['Times'],
        'axes.labelsize': 16,
        'axes.titlesize': 18,
        'legend.fontsize': 14,
        'xtick.labelsize': 14,
        'ytick.labelsize': 14,
        'lines.linewidth': 2,
        'axes.grid': True,
        'grid.alpha': 0.25,
    })

    # 1. Step response
    fig = plt.figure(figsize=(6, 4), dpi=300)
    plt.plot(t, y_step, '--', label=r'\textbf{Saída do Sistema}', color='black')
    plt.title(r'\textbf{Resposta ao Degrau}', pad=12)
    plt.xlabel(r'\textbf{Tempo (s)}')
    plt.ylabel(r'\textbf{Saída }$y(t)$')
    plt.legend(loc='best')
    plt.grid(True)
    plt.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, 'step_response.png'), dpi=300)
    plt.close(fig)

    # 2. PID vs Smith predictor
    fig, axs = plt.subplots(1, 2, figsize=(12, 5), dpi=300)

    # Output comparison
    axs[0].plot(t, y_pid, label=r'\textbf{PID}', color='navy')
    axs[0].plot(t, y_smith, label=r'\textbf{Smith Predictor}', color='darkgreen')
    if set_point is not None:
        axs[0].axhline(y=set_point, color='red', linestyle='--', label=r'\textbf{Referência}')
    axs[0].set_title(r'\textbf{Comparação da Saída}', pad=10)
    axs[0].set_xlabel(r'\textbf{Tempo (s)}')
    axs[0].set_ylabel(r'\textbf{Saída }$y(t)$')
    axs[0].legend(loc='best')
    axs[0].grid(True)

    # Control signals
    axs[1].plot(t, u_pid, label=r'\textbf{PID}', color='navy')
    axs[1].plot(t, u_smith, label=r'\textbf{Smith Predictor}', color='darkgreen')
    axs[1].set_title(r'\textbf{Sinais de Controle}', pad=10)
    axs[1].set_xlabel(r'\textbf{Tempo (s)}')
    axs[1].set_ylabel(r'\textbf{Controle }$u(t)$')
    axs[1].legend(loc='best')
    axs[1].grid(True)

    plt.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, 'pid_vs_smith.png'), dpi=300)
    plt.close(fig)
