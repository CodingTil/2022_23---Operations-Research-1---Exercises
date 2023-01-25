import numpy as np
import matplotlib.pyplot as plt


def display_solution(nodes, cleaning_arcs, transport_arcs, x):
    fig, ax = plt.subplots(1, 1, subplot_kw={'projection': 'polar'})
    displ_cleaning_arcs = []
    for a in cleaning_arcs:
        if (a[1], a[0]) not in displ_cleaning_arcs:
            displ_cleaning_arcs.append(a)
            i = nodes[a[0]]
            j = nodes[a[1]]
            if cleaning_arcs[a]['closing']:
                if j[0] == 0:
                    j = (360, j[1])
                elif j[0] == 0:
                    j = (360, j[1])
            theta = np.linspace(i[0], j[0], 100)
            r = np.linspace(i[1], j[1], 100)
            ax.plot(np.deg2rad(theta), r, color='blue', linestyle='-', linewidth=7, alpha=0.1,
                    label='Cleaning arcs' if len(displ_cleaning_arcs) == 1 else None)
    displ_transport_arcs = []
    for a in transport_arcs:
        if (a[1], a[0]) not in displ_transport_arcs:
            displ_transport_arcs.append(a)
            i = nodes[a[0]]
            j = nodes[a[1]]
            if i == (0, 0):
                i = (j[0], i[1])
            elif j == (0, 0):
                j = (i[0], j[1])
            if transport_arcs[a]['closing']:
                if i[0] == 0:
                    i = (360, i[1])
                elif j[0] == 0:
                    j = (360, j[1])
            theta = np.linspace(i[0], j[0], 100)
            r = np.linspace(i[1], j[1], 100)
            ax.plot(np.deg2rad(theta), r, color='red', linestyle='-', linewidth=7, alpha=0.1,
                    label='Transport arcs' if len(displ_transport_arcs) == 1 else None)
    for i in nodes:
        ax.scatter(np.deg2rad(nodes[i][0]), nodes[i][1], color='black', marker='o', zorder=3, s=30,
                   label='Nodes' if i == list(nodes.keys())[0] else None)
    colors = {0: 'green', 1: 'yellow', 2: 'orange', 3: 'cyan', 4: 'brown'}
    robot_used = {i: False for i in range(len(colors))}
    for a, k in x:
        if int(x[a, k].x) == 1:
            i = nodes[a[0]]
            j = nodes[a[1]]
            closing = False
            if a in cleaning_arcs:
                if cleaning_arcs[a]['closing']:
                    closing = True
            elif transport_arcs[a]['closing']:
                closing = True
            if closing:
                if j[0] == 0:
                    j = (360, j[1])
                elif i[0] == 0:
                    i = (360, i[1])
            if i[1] == 0:
                i = (j[0], 0)
            elif j[1] == 0:
                j = (i[0], 0)
            theta = np.linspace(i[0], j[0], 100)
            r = np.linspace(i[1], j[1], 100)
            ax.plot(np.deg2rad(theta), r, color=colors[k], linestyle='dashed', linewidth=2.5, alpha=1,
                    label=f'Tour robot {k}' if not robot_used[k] else None)
            if not robot_used[k]:
                robot_used[k] = True
    ax.set_yticklabels([])
    ax.legend(loc='lower center', bbox_to_anchor=(0.5, 1.1), ncol=3)
    plt.tight_layout()
    plt.show()
