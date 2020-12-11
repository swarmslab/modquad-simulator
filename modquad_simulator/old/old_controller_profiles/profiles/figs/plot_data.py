#!/usr/bin/python3

import numpy as np
import pandas as pd
import json
import matplotlib.pyplot as plt

outdir = "./"

def convert_hashstr_to_mat(sstr):
    rows = [x for x in sstr.split(";")]

    if len(rows) <= 0:
        raise Exception("No modules in structure?")

    num_rows = len(rows)
    num_cols = len(rows[0])
    num_mods = 0

    mat = np.zeros((num_rows, num_cols))
    for r, row in enumerate(rows):
        for c, col in enumerate(row):
            if col == "1":
                mat[r, c] = 1
                num_mods += 1

    # Assign mod IDs going from bottom right to top left
    cur_id = 1
    for r in range(num_rows - 1, -1, -1):
        for c in range(num_cols - 1, -1, -1):
            if mat[r][c] == 1:
                mat[r][c] = cur_id
                cur_id += 1

    return mat

def plot_for_file(fname):
    with open(fname, "r") as f:
        jdat = json.load(f) # Load json dict of data

    # Convert into plottable form
    for struc in jdat: # Loop over structures
        fdat = jdat[struc]
        #ofile = outdir + fdat + ".pdf"

        num_mods = len(jdat[struc].keys())
        num_rots = 4*num_mods
        
        # Get the matrix of assigned modules
        mat = convert_hashstr_to_mat(struc)

        print(mat)

        # Get the center index
        cx = int(mat.shape[0] / 2)
        cy = int(mat.shape[1] / 2)

        for rot in range(4):
            # Plot as we move along the x axis
            x_phi_d_min   = []
            x_phi_d_max   = []
            x_theta_d_min = []
            x_theta_d_max = []

            # NOTE: y = cy for all of this
            for i in range(0, mat.shape[0], 1):

                if str(int(mat[i, cy])) not in fdat:
                    continue

                x_phi_d_min.append  (fdat[str(int(mat[i, cy]))]["0"][0][0])
                x_phi_d_max.append  (fdat[str(int(mat[i, cy]))]["0"][1][0])
                x_theta_d_min.append(fdat[str(int(mat[i, cy]))]["0"][0][1])
                x_theta_d_max.append(fdat[str(int(mat[i, cy]))]["0"][1][1])

                #x_phi_d_min.append  (fdat[str(int(mat[i, cy]))]["1"][0][0])
                #x_phi_d_max.append  (fdat[str(int(mat[i, cy]))]["1"][1][0])
                #x_theta_d_min.append(fdat[str(int(mat[i, cy]))]["1"][0][1])
                #x_theta_d_max.append(fdat[str(int(mat[i, cy]))]["1"][1][1])

                #x_phi_d_min.append  (fdat[str(int(mat[i, cy]))]["2"][0][0])
                #x_phi_d_max.append  (fdat[str(int(mat[i, cy]))]["2"][1][0])
                #x_theta_d_min.append(fdat[str(int(mat[i, cy]))]["2"][0][1])
                #x_theta_d_max.append(fdat[str(int(mat[i, cy]))]["2"][1][1])

                #x_phi_d_min.append  (fdat[str(int(mat[i, cy]))]["3"][0][0])
                #x_phi_d_max.append  (fdat[str(int(mat[i, cy]))]["3"][1][0])
                #x_theta_d_min.append(fdat[str(int(mat[i, cy]))]["3"][0][1])
                #x_theta_d_max.append(fdat[str(int(mat[i, cy]))]["3"][1][1])

            plt.figure()
            plt.plot(x_phi_d_min, "b"   , label=r"$\dot{\phi}_{res,min}$")
            plt.plot(x_phi_d_max, "b:"  , label=r"$\dot{\phi}_{res,max}$")
            plt.plot(x_theta_d_min, "r" , label=r"$\dot{\theta}_{res,min}$")
            plt.plot(x_theta_d_max, "r:", label=r"$\dot{\theta}_{res,max}$")
            plt.legend()
            plt.tight_layout()
            plt.savefig(struc + "_left_to_right_{:01d}.pdf".format(rot))
            plt.close()

            # Plot as we move along the y axis
            y_phi_d_min   = []
            y_phi_d_max   = []
            y_theta_d_min = []
            y_theta_d_max = []
            # NOTE: x = cx for all of this
            for i in range(0, mat.shape[1], 1):

                if str(int(mat[cx, i])) not in fdat:
                    continue

                y_phi_d_min.append  (fdat[str(int(mat[cx, i]))]["0"][0][0])
                y_phi_d_max.append  (fdat[str(int(mat[cx, i]))]["0"][1][0])
                y_theta_d_min.append(fdat[str(int(mat[cx, i]))]["0"][0][1])
                y_theta_d_max.append(fdat[str(int(mat[cx, i]))]["0"][1][1])

                #y_phi_d_min.append  (fdat[str(int(mat[cx, i]))]["1"][0][0])
                #y_phi_d_max.append  (fdat[str(int(mat[cx, i]))]["1"][1][0])
                #y_theta_d_min.append(fdat[str(int(mat[cx, i]))]["1"][0][1])
                #y_theta_d_max.append(fdat[str(int(mat[cx, i]))]["1"][1][1])

                #y_phi_d_min.append  (fdat[str(int(mat[cx, i]))]["2"][0][0])
                #y_phi_d_max.append  (fdat[str(int(mat[cx, i]))]["2"][1][0])
                #y_theta_d_min.append(fdat[str(int(mat[cx, i]))]["2"][0][1])
                #y_theta_d_max.append(fdat[str(int(mat[cx, i]))]["2"][1][1])

                #y_phi_d_min.append  (fdat[str(int(mat[cx, i]))]["3"][0][0])
                #y_phi_d_max.append  (fdat[str(int(mat[cx, i]))]["3"][1][0])
                #y_theta_d_min.append(fdat[str(int(mat[cx, i]))]["3"][0][1])
                #y_theta_d_max.append(fdat[str(int(mat[cx, i]))]["3"][1][1])

            plt.figure()
            plt.plot(y_phi_d_min, "b"   , label=r"$\dot{\phi}_{res,min}$")
            plt.plot(y_phi_d_max, "b:"  , label=r"$\dot{\phi}_{res,max}$")
            plt.plot(y_theta_d_min, "r" , label=r"$\dot{\theta}_{res,min}$")
            plt.plot(y_theta_d_max, "r:", label=r"$\dot{\theta}_{res,max}$")
            plt.legend()
            plt.tight_layout()
            plt.savefig(struc + "_top_to_bottom_{:01d}.pdf".format(rot))
            plt.close()

            # Plot as we move along the long diag starting at top left
            # NOTE: THIS ONLY WORKS FOR SQUARE STRUCTURES
            diagR_phi_d_min   = []
            diagR_phi_d_max   = []
            diagR_theta_d_min = []
            diagR_theta_d_max = []
            # NOTE: y = cy for all of this
            for i in range(0, mat.shape[1], 1):

                if str(int(mat[i, i])) not in fdat:
                    continue

                diagR_phi_d_min.append  (fdat[str(int(mat[i, i]))]["0"][0][0])
                diagR_phi_d_max.append  (fdat[str(int(mat[i, i]))]["0"][1][0])
                diagR_theta_d_min.append(fdat[str(int(mat[i, i]))]["0"][0][1])
                diagR_theta_d_max.append(fdat[str(int(mat[i, i]))]["0"][1][1])

                #diagR_phi_d_min.append  (fdat[str(int(mat[i, i]))]["1"][0][0])
                #diagR_phi_d_max.append  (fdat[str(int(mat[i, i]))]["1"][1][0])
                #diagR_theta_d_min.append(fdat[str(int(mat[i, i]))]["1"][0][1])
                #diagR_theta_d_max.append(fdat[str(int(mat[i, i]))]["1"][1][1])

                #diagR_phi_d_min.append  (fdat[str(int(mat[i, i]))]["2"][0][0])
                #diagR_phi_d_max.append  (fdat[str(int(mat[i, i]))]["2"][1][0])
                #diagR_theta_d_min.append(fdat[str(int(mat[i, i]))]["2"][0][1])
                #diagR_theta_d_max.append(fdat[str(int(mat[i, i]))]["2"][1][1])

                #diagR_phi_d_min.append  (fdat[str(int(mat[i, i]))]["3"][0][0])
                #diagR_phi_d_max.append  (fdat[str(int(mat[i, i]))]["3"][1][0])
                #diagR_theta_d_min.append(fdat[str(int(mat[i, i]))]["3"][0][1])
                #diagR_theta_d_max.append(fdat[str(int(mat[i, i]))]["3"][1][1])

            plt.figure()
            plt.plot(diagR_phi_d_min, "b"   , label=r"$\dot{\phi}_{res,min}$")
            plt.plot(diagR_phi_d_max, "b:"  , label=r"$\dot{\phi}_{res,max}$")
            plt.plot(diagR_theta_d_min, "r" , label=r"$\dot{\theta}_{res,min}$")
            plt.plot(diagR_theta_d_max, "r:", label=r"$\dot{\theta}_{res,max}$")
            plt.legend()
            plt.tight_layout()
            plt.savefig(struc + "_diag_from_top_right_{:01d}.pdf".format(rot))
            plt.close()

            # Plot as we move along the long diag starting at top left
            # NOTE: THIS ONLY WORKS FOR SQUARE STRUCTURES
            diagL_phi_d_min   = []
            diagL_phi_d_max   = []
            diagL_theta_d_min = []
            diagL_theta_d_max = []
            # NOTE: y = cy for all of this
            for i,j in list(zip(range(mat.shape[1]-1,-1,-1), range(0, mat.shape[0]))):

                if str(int(mat[i, j])) not in fdat:
                    continue

                diagL_phi_d_min.append  (fdat[str(int(mat[i, j]))]["0"][0][0])
                diagL_phi_d_max.append  (fdat[str(int(mat[i, j]))]["0"][1][0])
                diagL_theta_d_min.append(fdat[str(int(mat[i, j]))]["0"][0][1])
                diagL_theta_d_max.append(fdat[str(int(mat[i, j]))]["0"][1][1])

                #diagL_phi_d_min.append  (fdat[str(int(mat[i, j]))]["1"][0][0])
                #diagL_phi_d_max.append  (fdat[str(int(mat[i, j]))]["1"][1][0])
                #diagL_theta_d_min.append(fdat[str(int(mat[i, j]))]["1"][0][1])
                #diagL_theta_d_max.append(fdat[str(int(mat[i, j]))]["1"][1][1])

                #diagR_phi_d_min.append  (fdat[str(int(mat[i, j]))]["2"][0][0])
                #diagR_phi_d_max.append  (fdat[str(int(mat[i, j]))]["2"][1][0])
                #diagR_theta_d_min.append(fdat[str(int(mat[i, j]))]["2"][0][1])
                #diagR_theta_d_max.append(fdat[str(int(mat[i, j]))]["2"][1][1])

                #diagR_phi_d_min.append  (fdat[str(int(mat[i, j]))]["3"][0][0])
                #diagR_phi_d_max.append  (fdat[str(int(mat[i, j]))]["3"][1][0])
                #diagR_theta_d_min.append(fdat[str(int(mat[i, j]))]["3"][0][1])
                #diagR_theta_d_max.append(fdat[str(int(mat[i, j]))]["3"][1][1])

            plt.figure()
            plt.plot(diagL_phi_d_min, "b"   , label=r"$\dot{\phi}_{res,min}$")
            plt.plot(diagL_phi_d_max, "b:"  , label=r"$\dot{\phi}_{res,max}$")
            plt.plot(diagL_theta_d_min, "r" , label=r"$\dot{\theta}_{res,min}$")
            plt.plot(diagL_theta_d_max, "r:", label=r"$\dot{\theta}_{res,max}$")
            plt.legend()
            plt.tight_layout()
            plt.savefig(struc + "_diag_from_bottom_right_{:01d}.pdf".format(rot))

            plt.close()

            print("------------------------------------------")

if __name__ == "__main__":
    filename = "../fprof_ranged.json"
    plot_for_file(filename)
