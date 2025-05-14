from sklearn.metrics import confusion_matrix
import csv
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np

NUMBER_OF_CLASSES = 7
log_name = 'real_crnn250de3900de1300dr25_3_5_out'

def resize_cf(matrix, labels):

    resized_matrix=matrix.copy().astype(float)
    #print(resized_matrix)

    for i in range(resized_matrix.shape[0]):
        where_labels = np.where(np.array(labels)==i)[0]
        overall = np.sum(where_labels.shape[0])
        #print(overall)
        resized_matrix[i,:] = resized_matrix[i,:]/overall

    return resized_matrix.copy()

with open('src/networks/' + log_name + "_For_confusions.csv") as csv_file:
    csv_reader = csv.reader(csv_file,delimiter = ',')

    rows = [row for idx, row in enumerate(csv_reader)]

    ground_truth = list(rows[0][0].split(", "))
    ground_truth[0] = ground_truth[1]
    ground_truth[-1] = ground_truth[-2]
    ground_truth = list(map(int,ground_truth))
    untrained_predictions = list(rows[0][1].split(", "))
    untrained_predictions[0] = untrained_predictions[1]
    untrained_predictions[-1] = untrained_predictions[-2]
    untrained_predictions = list(map(int,untrained_predictions))

    print(ground_truth)

    print(untrained_predictions)

    cf1 = confusion_matrix(ground_truth, untrained_predictions)


    categories = ['RR', 'WW', 'WP', 'WR', 'PW', 'RW', 'PP']

    group_counts = ["{0:0.0f}".format(value) for value in
                    cf1.flatten()]

    group_percentages = ["{0:.2%}".format(value) for value in
                        cf1.flatten()/np.sum(cf1)]

    labels = [f"{v1}\n{v2}\n" for v1, v2 in
            zip(group_counts,group_percentages)]

    labels = np.asarray(labels).reshape(NUMBER_OF_CLASSES, NUMBER_OF_CLASSES)

    a_cf1 = cf1.astype('float') / cf1.sum(axis=1)[:, np.newaxis]

    ax = sns.heatmap(a_cf1, annot=True, fmt='.2', cmap='Blues',vmin = 0, vmax = 1,xticklabels=categories,yticklabels=categories)
    plt.show()