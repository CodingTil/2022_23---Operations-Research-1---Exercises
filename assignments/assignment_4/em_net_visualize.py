import matplotlib.pyplot as plt


def display_solution(x, y, locations, transport_methods, reaction_times):
    fig, ax = plt.subplots(1, 1)
    labels = {i: False for i in ['amb', 'hel', 'free', 'con_amb', 'con_hel']}
    for i in locations:
        if int(y[i, 'hel'].x) == 1:
            ax.scatter(locations[i][0], locations[i][1],
                       color='blue',
                       facecolors='blue',
                       marker='o',
                       label='Hosp. with amb. and hel.' if not labels['hel'] else None,
                       zorder=1)
            labels['hel'] = True
        elif int(y[i, 'amb'].x) == 1:
            ax.scatter(locations[i][0], locations[i][1],
                       color='red',
                       facecolors='red',
                       marker='o',
                       label='Hosp. with amb.' if not labels['amb'] else None,
                       zorder=1)
            labels['amb'] = True
        else:
            ax.scatter(locations[i][0], locations[i][1],
                       color='black',
                       facecolors='white',
                       marker='o',
                       label='No hosp.' if not labels['free'] else None,
                       zorder=1)
            labels['free'] = True
        for j in locations:
            for t in transport_methods:
                if int(x[i, j, t].x) == 1:
                    if t == 'amb':
                        if labels['con_amb']:
                            label = None
                        else:
                            label = 'Con. with amb.'
                            labels['con_amb'] = True
                    elif labels['con_hel']:
                            label = None
                    else:
                        label = 'Con. with hel.'
                        labels['con_hel'] = True
                    ax.plot([locations[i][0], locations[j][0]],
                            [locations[i][1], locations[j][1]],
                            color='red' if t == 'amb' else 'blue',
                            linestyle='dotted' if t == 'amb' else 'dashed',
                            label=label,
                            zorder=0)
                    if locations[i][0] != locations[j][0] or locations[i][1] != locations[j][1]:
                        ax.text((locations[i][0] + locations[j][0]) / 2,
                                (locations[i][1] + locations[j][1]) / 2,
                                f'{reaction_times[i, j , t]}', bbox={'facecolor': 'white',  'alpha': 0.8, 'pad': 0.5})
    ax.legend(loc='lower center', bbox_to_anchor=(0.5, 1), ncol=3)
    plt.show()
