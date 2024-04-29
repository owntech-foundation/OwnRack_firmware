import glob 
import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np 


plt.rcParams['lines.linewidth'] = 3
plt.rcParams['figure.figsize'] = [15, 7]
plt.rcParams['font.size'] = 18
def plot_files_start_with(file_start_with):
    all_dfs = {}
    for k in ['c','b','a']:
        try:
            all_dfs[k] = pd.read_csv(f'{file_start_with}_{k}.csv')
            # all_dfs[k] = pd.read_csv(f'2024_01_18_25Hz_with_nidec_phase_{k}.csv')
        except:
            pass

    fig, axs = plt.subplots(2, 1, sharex=True)
    for k, df in all_dfs.items():
        axs[0].step(np.arange(len(df)), df['Vref'], label=f'Vref_{k}')
        axs[0].step(np.arange(len(df)), df['w'], label=f'w_{k}')
        #axs[1].step(np.arange(len(df)), df['duty_cycle'], label=f'control_task_{k}')
        axs[1].step(np.arange(len(df)), df['I1_low'], label=f'I1_low_{k}')
        axs[1].step(np.arange(len(df)), df['I2_low'], label=f'I2_low_{k}')

    for ax in axs:
        ax.legend()
        ax.grid()
#        ax.set_xlim([0, 500])

    axs[0].set_title('Vrefs received by RS485 from ownverter')
    # axs[1].set_ylabel('time [us]')
    axs[1].set_title('Currents transmit to ownverter using RS485')
    axs[1].set_xlabel('call number of the control task')
    axs[1].set_ylabel('current [A]')
    axs[0].set_ylabel('voltage ref [V]')
    #fig.suptitle(file_start_with.replace('_nidec_propre',''))
    # fig.suptitle('Datas exchanged between OwnTech cards during three-phase motor control')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    import sys
    if len(sys.argv)<2:
        raise ValueError("should have one argument")
    else:
        a_string = sys.argv[1]
        plot_files_start_with(a_string)
