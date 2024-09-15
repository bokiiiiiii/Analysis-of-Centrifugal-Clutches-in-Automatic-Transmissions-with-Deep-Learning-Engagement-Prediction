import numpy as np
import pandas as pd
import tensorflow as tf



## Data Preprocessing
# Importing the dataset
dataset = pd.read_excel('prediect_result_cofig_b.xlsx')
X = dataset.iloc[1:, :3].values
Y = dataset.iloc[1:, 4].values
Y = np.where(Y == 0, 0, 1)

# Splitting the dataset into the Training set and Test set
from sklearn.model_selection import train_test_split
X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size = 0.2, random_state = 0)

X_train = np.array(X_train, dtype=np.float32)
Y_train = np.array(Y_train, dtype=np.float32)

# # Feature Scaling
import joblib
scx = joblib.load('scaler.pkl')
X_train = scx.fit_transform(X_train)
X_test = scx.transform(X_test)


# Loading the model from the file
loaded_model = tf.keras.models.load_model('config_b_DNN_model.h5')

# Making predictions with the loaded model
Y_pred = loaded_model.predict(X_test)
Y_pred = np.round(Y_pred)


# Visualising the Test set results in black and white
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np

X_set, y_set = scx.inverse_transform(X_test), Y_test

X1, X2 = np.meshgrid(np.arange(start = X_set[:, 0].min() - 45.0, stop = X_set[:, 0].max() + 45.0, step = 0.0450),
                     np.arange(start = X_set[:, 1].min() - 0.01, stop = X_set[:, 1].max() + 0.01, step = 0.0005))


plt.contourf(X1, X2, loaded_model.predict(scx.transform(np.array([X1.ravel(), X2.ravel(), np.ones_like(X1.ravel())]).T)).reshape(X1.shape),
             alpha = 0.75, cmap = ListedColormap(('white', 'black')))

plt.xlim(X1.min(), X1.max())
plt.ylim(X2.min(), X2.max())


# Set the font to Times New Roman
from matplotlib import rcParams
rcParams['font.family'] = 'serif'
rcParams['font.serif'] = ['Times New Roman'] 

for i, label in enumerate(['Not engaged', 'Engaged']):
    plt.scatter(X_set[y_set == i, 0], X_set[y_set == i, 1], 
                c = ListedColormap(('white', 'black'))(i), 
                edgecolor='black', label=label, s=10)


plt.title(r'Deep Learning Based Centrifugal Clutch Engagement Prediction', 
          fontdict={'family': 'Times New Roman', 'size': 14, 'weight': 'bold'})
plt.xlabel(r'$F_{\mathrm{preload}}$ ${(N)}$') 
plt.ylabel(r'$m_{\mathrm{shoe}}$ ${(g)}$')

plt.xticks(fontsize=9.5)
plt.yticks(fontsize=9.5)

plt.legend(loc='lower right', labelspacing=0.1, handletextpad=0.2, framealpha=1.0) 

plt.show()
