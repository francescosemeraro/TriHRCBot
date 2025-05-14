import numpy as np
import pickle

class Dataset:
    def __init__(self,data_points=[],labels=[],prep_data=[],test_set=[],test_labels=[],test_prep=[]):
        self.dataset = [data_points,labels,prep_data]
        self.test_set = [test_set, test_labels,test_prep]

    def append_data(self,entries):
        '''Appends a data point, with the related label and additional information, to the dataset collection.

        Parameters:
            List constituted by a list of the data points themselves, a list of their labels and a list of additional information.

        Returns: 
            None.
        '''
        for i in range(len(entries[0])):
            self.dataset[0].append(entries[0][i])
            self.dataset[1].append(entries[1][i])
            if entries[2] != []:
                self.dataset[2].append(entries[2][i])
    
    def append_test(self,entries):
        '''Appends a data point, with the related label and additional information, to the test set collection.

        Parameters:
            List constituted by a list of the data points themselves, a list of their labels and a list of additional information.

        Returns: 
            None.
        '''
        for i in range(len(entries[0])):
            self.test_set[0].append(entries[0][i])
            self.test_set[1].append(entries[1][i])
            if entries[2] != []:
                self.test_set[2].append(entries[2][i])

    def clear_dataset(self):
        '''Removes every data point from the Dataset object.

        Parameters:
            None.
            
        Returns: 
            None.
        '''
        self.dataset = [[],[],[]]
        self.test_set = [[],[],[]]

    def copy_dataset(self):
        '''Copies the dataset into an empty Dataset object, that is returned at the end of the method.

        Parameters:
            None.

        Returns:
            Dataset object, copy of the current one.
        
        '''
        a_copy=Dataset(self.dataset[0].copy(),self.dataset[1].copy(),self.dataset[2].copy(),self.test_set[0].copy(),self.test_set[1].copy(),self.test_set[2].copy())
        
        return a_copy

    def get_data(self):
        '''Gets the data points.

        Parameters:
            None.

        Returns: 
            List containing the data points.
        '''
        return self.dataset[0]

    def get_data_as_array(self):
        '''Gets the data points, in a shape of a NumPy array.

        Parameters:
            None.

        Returns: 
            NumPy array containing the data points.
        '''
        return np.array(self.dataset[0])
        
    def get_data_dim(self):
        '''Gets the dimensionality of a single data point.

        Parameters:
            None.

        Returns: 
            Tuple representing the dimensionality of a data point in the dataset.
        '''
        return self.get_data()[0].shape

    def get_labels(self):
        '''Gets the labels of the dataset.

        Parameters:
            None.

        Returns: 
            A list with the labels of the dataset.
        '''
        return self.dataset[1]

    def get_first_label(self):
        '''Gets the first label in the dataset.

        Parameters:
            None.

        Returns: 
            A list with the labels of the dataset.
        '''
        return self.dataset[1][0]

    def get_points(self, list_of_points):

        returned_points = [[],[],[]]

        for point in list_of_points:
            returned_points[0].append(self.dataset[0][point])
            returned_points[1].append(self.dataset[1][point])
            if self.dataset[2]!=[]:
                returned_points[2].append(self.dataset[2][point])

        return Dataset(data_points=returned_points[0],labels=returned_points[0],prep_data=returned_points[0],test_set=[],test_labels=[],test_prep=[])

    def get_prep(self):
        '''Gets the pre-processing data of the dataset.

        Parameters:
            None.

        Returns: 
            List containing the pre-processing data of the dataset.
        '''
        return self.dataset[2]

    def get_labels_as_array(self):
        '''Gets the labels of the dataset, in a shape of a NumPy array, if applicable.

        Parameters:
            None.

        Returns: 
            NumPy array containing the labels of the dataset, if applicable.
        '''
        return np.array(self.dataset[1])
    
    def get_size(self):
        '''Gets the number of data points.

        Parameters:
            None.

        Returns: 
            Integer representing the number of data points in the dataset.
        '''
        return len(self.dataset[0])

    def get_test(self):
        '''Gets the test set of the dataset.

        Parameters:
            None.

        Returns: 
            List representing the test set of the dataset.
        '''
        return self.test_set[0]
    
    def get_test_as_array(self):
        '''Gets the test set of the dataset, in the shape of a NumPy array.

        Parameters:
            None.

        Returns: 
            NumPy array containing the test set of the dataset.
        '''
        return np.array(self.test_set[0])

    def get_test_dim(self):
        '''Gets the dimensionality of a single data point.

        Parameters:
            None.

        Returns: 
            Tuple representing the dimensionality of a data point in the dataset.
        '''
        return self.get_data_as_array().shape[1:]
    
    def get_test_labels(self):
        '''Gets the labels of the test set of the dataset.

        Parameters:
            None.

        Returns: 
            Tuple representing the dimensionality of a data point in the dataset.
        '''
        return self.test_set[1]

    def get_test_labels_as_array(self):
        '''Gets the labels of the test set, in a shape of a NumPy array, if applicable.

        Parameters:
            None.

        Returns: 
            NumPy array containing the labels of the test set of the dataset, if applicable.
        '''
        return np.array(self.test_set[1])

    def get_test_size(self):
        '''Gets the number of instances of the training set.

        Parameters:
            None.

        Returns: 
            Integer representing the number of data points in the dataset.
        '''
        return len(self.test_set[0])

    def is_balanced(self):
        '''Prints information about classes' names and the number of data points for each of them.

        Parameters:
            None.

        Returns: 
            None.
        '''
        classes = list(set(self.get_labels()))
        
        numbers = [0] * len(classes) 

        for i in self.get_labels():
            numbers[i] = numbers[i] + 1 

        print('The dataset is made of: ')
        for i in range(len(numbers)):
            print('Class ', classes[i], ': ', numbers[i])

    def isolate_subject(self,tag,positions):
        '''Generates a test set composed of all the data points of the Dataset object associated to one tag. Such tag is present as part of a bigger tag 
        in a set of tags, in the most generic case. Therefore, the method needs to know where to look for the tag in every possible bigger tag provided.

        Parameters:
            String related to the subject to isolate in the test set, list of the indices where to look for your tag for each bigger tag provided.

        Returns: 
            None.
        '''
        appended = False
        sampled=[]

        for i in range(self.get_size()):
            appended = False
            for j in positions:
                for k in self.dataset[2][i]:
                    if appended:
                        continue
                    elif k[j[0]:j[1]]==tag:
                        sampled.append(i)
                        self.append_test([[self.dataset[0][i]],[self.dataset[1][i]],[self.dataset[2][i]]])
                        appended = True

        self.pop('train',sampled)

    def k_fold_crossvalidation(self,k):
        '''Generates list of indices that can be used for a crossvalidation.

        Parameters:
            Integer representing the number of desired folds.

        Returns:
            List containing lists, each one containing a balanced fold of the original dataset.
        '''
        classes = set(self.dataset[1])

        number_of_classes = len(classes)

        number_of_data_points = len(self.dataset[0])

        fold_size = int(np.round(number_of_data_points/k))

        points_per_class_in_fold = int(round(fold_size/number_of_classes))

        to_exclude = []

        folds = []

        for i in range(k):
            sampled = []
            for j in range(number_of_classes):
                candidates = np.where(np.array(self.dataset[1])==j)[0]
                sampled.extend(np.random.choice(list(set(candidates)-set(to_exclude)),points_per_class_in_fold,replace=False))
                to_exclude.extend(sampled)
            folds.append(sampled)
        
        return folds

    def load(self, file_name=''):
        '''Loads a Dataset object and uses it to initialise the original one. It will be loaded the same path of where this method is called.
        If the dataset is not empty, it will invoke the sum(new_dataset) method.

        Parameters:
            Name of the file to load, that must include '.pickle' at the end

        Returns:
            None.
        
        '''
        
        dataset = pickle.load(open(file_name, 'rb'))
        dataset = dataset[0]
        
        if dataset.get_size()==0: #Assumption of never having just a test set to load
            dummy = Dataset([],[],[],[],[],[])
            dummy = self.unite(dataset)
            self.dataset = [dummy.get_data(),dummy.get_labels()]
            self.test_set = [dummy.get_test(), dummy.get_test_labels()]            
        else:
            self.append_data(dataset.dataset)
            
            if dataset.test_set != [[],[],[]]:
                self.append_test(dataset.test_set)

    def make_test_set(self, points_per_class):
        '''Generates a test set that will be stored in the Dataset object itself. It selects the same number of points for each class in the dataset.

        Parameters:
            Integer representing the desired number of data points per class.

        Returns:
            None.
        '''
        classes = set(self.dataset[1])

        sampled = []

        for i in range(len(classes)):
            sampled.extend(np.random.choice(np.where(np.array(self.dataset[1])==i)[0],points_per_class,replace=False))

        for i in sampled:
            self.append_test([[self.dataset[0][i]],[self.dataset[1][i]],[self.dataset[2][i]]])
        
        self.pop('train',sampled)
 
    def merge(self,dataset_to_merge,tag, tag_to_merge):
        '''Merges two Dataset objects together, generating a new Dataset object with pairs of data points from the two datasets merged in an unique instance.
        It will not make any dimensionality check for data consistency, so be careful, especially in giving datasets with the same number of instances.

        Parameters:
            Dataset objects to be merged, a string representing an information related to the data to merge, a string representing an information 
            related to the data to be merged. 

        Returns: 
            Dataset object, that represents the merging of the two Dataset objects involved.
        '''
        merged_dataset= Dataset([],[],[],[],[],[])
        size = self.get_data_dim()
        data_merger = np.zeros((size[0],2*size[1],size[2]))
        data = []
        labels = []
        preps = []

        for i in range(self.get_size()):

            for j in range(dataset_to_merge.get_size()):
                data_merger[:,0:size[1],:]=self.get_data_as_array()[i]
                data_merger[:,size[1]:,:]=dataset_to_merge.get_data_as_array()[j]
                data.append(data_merger)
                preps.append([tag,tag_to_merge])
                labels.append(self.select_case(self.get_label(),dataset_to_merge.get_label()))
                
        merged_dataset.append_data([data,labels,preps])

        return merged_dataset

    def min_max(self, old_min, old_max, new_min, new_max):
        '''Applies a min-max normalisation to the Dataset object, not considering the test set.

        Parameters:
            Old minimum and maximum, new minimum and maximum requested.

        Returns: 
            None.
        '''
        for i in range(self.get_size()):
            self.dataset[0][i] = ((self.dataset[0][i]-old_min)/(old_max-old_min)) * (new_max-new_min) + new_min
        
    def pop(self,set,indexes):
        '''Pops out elements of the training set or test set, according to what is specified.

        Parameters:
            A string representing the set to pop elements from, a list of the indexes of the elements to discard.

        Returns: 
            None.
        '''
        indexes = sorted(indexes)

        for i in range(len(indexes)):
            indexes[i]=indexes[i]-i

        if set == 'train':
            for index in indexes:
                self.dataset[0].pop(index)
                self.dataset[1].pop(index)
                if self.dataset[2] != []:
                    self.dataset[2].pop(index)
        elif set == 'test':
            for index in indexes:
                self.test_set[0].pop(index)
                self.test_set[1].pop(index)
                if self.test_set[2] != []:
                    self.test_set[2].pop(index)

    def rename_classes(self):
        '''Remaps classes to adjacent integers, to make it fit criteria for tf.utils.to_categorical().

        Parameters:
            None.

        Returns:
            None.
        '''
        labels = self.get_labels_as_array()

        classes = list(set(labels))

        for i in range(len(classes)):
            labels[labels==classes[i]]=i

        self.dataset[1] = labels.tolist()
   
    def remove_class(self,label):
        '''Removes all the data points in the training set with the same label.
        
        Parameters:
            An integer, representing the label to remove from the dataset.

        Returns: 
            None.
        '''
        data_to_remove=np.where(self.get_labels_as_array()==label)
        self.pop('train',data_to_remove[0].tolist())

    def save(self, filename=''):
        '''Saves the Dataset object in a pickle file format. It will be saved in the same path of where this method is called.

        Parameters:
            Name of the file where to store the data, that must include '.pickle' at the end

        Returns:
            None.
        
        '''
        with open(filename, "wb") as output:  # Overwrites any existing file. 
            pickle.dump([Dataset(data_points=self.dataset[0],labels=self.dataset[1],prep_data=self.dataset[2],
            test_set=self.test_set[0],test_labels=self.test_set[1],test_prep=self.test_set[2])], output)#It has to be done this way, otherwise pickle 
                                                                                                            #replaces the Dataset object with a list, 
                                                                                                            #losing all the built-in methods

    def select_case(self,label1, label2):
        '''Gives the merged data point a new label, by looking at the labels of the original data points merged.

        Parameters:
            The labels of the two original data points.

        Returns: 
            An integer representing the new label.
        '''
        if label1 == 1 and label2 == 1: 
            return 1 #Working-Working
        elif label1 ==1 and label2 ==2:
            return 2 #Working-Requesting
        elif label1 ==1 and label2 ==3:
            return 3 #Working-Preparing
        elif label1 ==2 and label2 ==1:
            return 4 #Requesting-Working
        elif label1 ==2 and label2 ==2:
            return 5 #Requesting-Requesting
        elif label1 ==2 and label2 ==3:
            return 6 #Requesting-Preparing
        elif label1 ==3 and label2 ==1:
            return 7 #Preparing-Working
        elif label1 ==3 and label2 ==2:
            return 8 #Preparing-Requesting
        elif label1 ==3 and label2 ==3:
            return 9 #Preparing-Preparing
    
    def shrink_dataset(self, points_per_class):
        '''Reduces the size of the Dataset object, by randomly retaining the requested amount of data points per class and deleting the remaining ones.

        Parameters:
            Number of points to retain from each class.

        Returns: 
            None.
        '''
        classes = set(self.dataset[1])

        sampled = []

        numbers = [0] * len(classes) 

        for i in self.get_labels():
            numbers[i] = numbers[i] + 1 

        for i in range(len(classes)):
            selection = np.random.choice(np.where(self.get_labels_as_array()==i)[0],numbers[i]-points_per_class,replace=False)
            sampled.extend(selection)
        
        self.pop('train',sampled)

    def unite(self,new_dataset):
        '''Unites the current Dataset object with another Dataset object, generating a new Dataset object with all the data points, labels and additional
        information of the two objects. It will not make any dimensionality check for data consistency, so be careful.

        Parameters:
            One of the two Dataset objects to unite.

        Returns: 
            None.
        '''
        self.append_data(new_dataset.dataset)
        
        if new_dataset.test_set != [[],[],[]]:
            self.append_test(new_dataset.test_set)
        
