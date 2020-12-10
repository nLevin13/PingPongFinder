import numpy as np
from sklearn import linear_model
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error


simdata = np.genfromtxt('simdata2.csv', delimiter=',')
locations = simdata[:, :3]
intensities = simdata[:, 3:]

#X = intensities
X = np.concatenate((intensities**(-1), intensities), axis=1)
#X = np.concatenate((intensities**(-1), intensities**(-2), intensities), axis=1)
#X = np.log2(intensities**(-2))
Y = locations

X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.5, random_state=0)
regr = linear_model.LinearRegression()
#regr.fit(X_train, Y_train)
regr.fit(X, Y)

print("R2 value: ", regr.score(X_test, Y_test))
print("Mean Squared Error: ", mean_squared_error(Y_test, regr.predict(X_test) ) )