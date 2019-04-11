from keras.models import Sequential
from keras.layers import Dense
from keras.models import model_from_json
from keras import optimizers
import numpy as np


json_file = open('model.json','r')
loaded_model_json = json_file.read()
json_file.close()
loaded_model = model_from_json(loaded_model_json)
loaded_model.load_weights("model.h5")
adam = optimizers.Adam(lr=0.001, decay =1e-6)
loaded_model.compile(loss='categorical_crossentropy', optimizer=adam, metrics=['accuracy'])
testY = np.load('groundTruth.npy')
testX = np.load('testData.npy')
score = loaded_model.evaluate(testX,testY,verbose=2)

print('Baseline Error: %.2f%%' %(100-score[1]*100))

output = loaded_model.predict(testX[0:1,:,:,:])

print("Predect Output" , output)

