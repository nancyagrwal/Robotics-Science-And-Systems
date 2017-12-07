% This is a variation on the LeNet convolutional neural network for MNIST
% digit classification. However, this version learns very slowly. Modify two
% lines of code so that it learns faster.
% 
% For more information, see the LeNet discussion here:
% https://www.mathworks.com/help/nnet/examples/create-simple-deep-learning-network-for-classification.html
%
% input: Null
% output: a matlab "Layer" object. For more info, see:
%         https://www.mathworks.com/help/nnet/ref/nnet.cnn.layer.layer.html
%
function layers = Q1()

    % ** Modify the code below **

    layers = [  imageInputLayer([28 28 1])

                convolution2dLayer(3,16,'Padding',1)
                batchNormalizationLayer
                reluLayer %reluLayer for a non-linear activation function:
                %softmaxLayer

                maxPooling2dLayer(2,'Stride',2)

                % added another covolution layer:
                convolution2dLayer(3,32,'Padding',1)%changed 64 to 32
                batchNormalizationLayer
                reluLayer %reluLayer for a non-linear activation function:
                %softmaxLayer

                maxPooling2dLayer(2,'Stride',2)

                convolution2dLayer(3,64,'Padding',1)
                batchNormalizationLayer
                reluLayer 

                %fullyConnectedLayer(50)
                %reluLayer

                % there are 10 classes, so output paramater in FCL should be 10:
                fullyConnectedLayer(10)
                softmaxLayer
                classificationLayer];
end
