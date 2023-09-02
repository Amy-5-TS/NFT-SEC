%matplotlib inline
import matplotlib.pyplot as plt
import numpy as np
from sklearn.linear_model import LinearRegression
model = LinearRegression(fit_intercept=True)

x = np.array([1,2,3,4,5,6,7,8])
y = np.array([1,2,3,4,5,6,7,8])
model.fit(x[:, np.newaxis], y)
xfit = np.linspace(x.item(0), x.item(len(x)-1), 1000)
yfit = model.predict(xfit[:, np.newaxis])
plt.scatter(x, y)
plt.plot(xfit, yfit)
print("Model slope:    ", model.coef_[0])
print("Model intercept:", model.intercept_)
data = np.array([999])
result = model.predict(data[:, np.newaxis])
print(result)