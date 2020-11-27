import tensorflow as tf
from tensorflow.keras.layers import Input, Conv2D, AveragePooling2D, Conv2DTranspose
from tensorflow.keras.layers import BatchNormalization, LeakyReLU, Dropout
from tensorflow.keras.models import Model
from tensorflow.keras.optimizers import Adam
from tensorflow.keras import backend as K

class ResContextBlock(tf.keras.Model):
    def __init__(self, out_filters):
        super(ResContextBlock, self).__init__()
        self.conv1 = Conv2D(out_filters, kernel_size=(1, 1), strides=1)
        self.act1 = LeakyReLU()

        self.conv2 = Conv2D(out_filters, kernel_size=(3,3), padding='same')
        self.act2 = LeakyReLU()
        self.bn1 = BatchNormalization()

        self.conv3 = Conv2D(out_filters, kernel_size=(3,3), dilation_rate=2, padding='same')
        self.act3 = LeakyReLU()
        self.bn2 = BatchNormalization()


    def call(self, x):

        shortcut = self.conv1(x)
        shortcut = self.act1(shortcut)

        resA = self.conv2(shortcut)
        resA = self.act2(resA)
        resA1 = self.bn1(resA)

        resA = self.conv3(resA1)
        resA = self.act3(resA)
        resA2 = self.bn2(resA)

        output = shortcut + resA2
        return output

class ResBlock(tf.keras.Model):
    def __init__(self, out_filters, dropout_rate, kernel_size=(3, 3), stride=1,
                 pooling=True, drop_out=True):
        super(ResBlock, self).__init__()
        self.pooling = pooling
        self.drop_out = drop_out
        self.conv1 = Conv2D(out_filters, kernel_size=(1, 1), strides=stride)
        self.act1 = LeakyReLU()

        self.conv2 = Conv2D(out_filters, kernel_size=(3, 3), padding='same')
        self.act2 = LeakyReLU()
        self.bn1 = BatchNormalization()

        self.conv3 = Conv2D(out_filters, kernel_size=(3,3),dilation_rate=2, padding='same')
        self.act3 = LeakyReLU()
        self.bn2 = BatchNormalization()

        self.conv4 = Conv2D(out_filters, kernel_size=(2, 2), dilation_rate=2, padding='same')
        self.act4 = LeakyReLU()
        self.bn3 = BatchNormalization()

        self.conv5 = Conv2D(out_filters, kernel_size=(1, 1))
        self.act5 = LeakyReLU()
        self.bn4 = BatchNormalization()

        if pooling:
            self.dropout = Dropout(dropout_rate)
            self.pool = AveragePooling2D(pool_size=kernel_size, strides=2, padding='same')
        else:
            self.dropout = Dropout(dropout_rate)

    def call(self, x):
        shortcut = self.conv1(x)
        shortcut = self.act1(shortcut)

        resA = self.conv2(x)
        resA = self.act2(resA)
        resA1 = self.bn1(resA)

        resA = self.conv3(resA1)
        resA = self.act3(resA)
        resA2 = self.bn2(resA)

        resA = self.conv4(resA2)
        resA = self.act4(resA)
        resA3 = self.bn3(resA)

        concat = tf.concat((resA1,resA2,resA3),axis=-1)
        resA = self.conv5(concat)
        resA = self.act5(resA)
        resA = self.bn4(resA)
        resA = shortcut + resA


        if self.pooling:
            if self.drop_out:
                resB = self.dropout(resA)
            else:
                resB = resA
            resB = self.pool(resB)
            return resB, resA
        else:
            if self.drop_out:
                resB = self.dropout(resA)
            else:
                resB = resA
            return resB

class UpBlock(tf.keras.Model):
    def __init__(self, out_filters, dropout_rate, drop_out=True):
        super(UpBlock, self).__init__()
        self.drop_out = drop_out
        self.out_filters = out_filters

        self.dropout1 = Dropout(dropout_rate)
        self.dropout2 = Dropout(dropout_rate)

        self.conv1 = Conv2D(out_filters, kernel_size=(3,3), padding='same')
        self.act1 = LeakyReLU()
        self.bn1 = BatchNormalization()

        self.conv2 = Conv2D(out_filters, kernel_size=(3,3),dilation_rate=2, padding='same')
        self.act2 = LeakyReLU()
        self.bn2 = BatchNormalization()

        self.conv3 = Conv2D(out_filters, kernel_size=(2,2), dilation_rate=2,padding='same')
        self.act3 = LeakyReLU()
        self.bn3 = BatchNormalization()


        self.conv4 = Conv2D(out_filters,kernel_size=(1,1))
        self.act4 = LeakyReLU()
        self.bn4 = BatchNormalization()
        self.dropout3 = Dropout(dropout_rate)

    def call(self, x, skip):
        upA = tf.nn.depth_to_space(x,2)
        if self.drop_out:
            upA = self.dropout1(upA)

        upB = tf.concat((upA,skip),axis=-1)
        if self.drop_out:
            upB = self.dropout2(upB)

        upE = self.conv1(upB)
        upE = self.act1(upE)
        upE1 = self.bn1(upE)

        upE = self.conv2(upE1)
        upE = self.act2(upE)
        upE2 = self.bn2(upE)

        upE = self.conv3(upE2)
        upE = self.act3(upE)
        upE3 = self.bn3(upE)

        concat = tf.concat((upE1,upE2,upE3),axis=-1)
        upE = self.conv4(concat)
        upE = self.act4(upE)
        upE = self.bn4(upE)
        if self.drop_out:
            upE = self.dropout3(upE)

        return upE

def LiDAR_Model(input_layers, output_layers):
    INPUT_SHAPE = [None,None,input_layers]
    # Input shape kitti dataset
    #INPUT_SHAPE = [2048,64,5]
    # input shape of bugalog data
    #INPUT_SHAPE = [1024,16,4]
    # min input shape
    #INPUT_SHAPE = [16,16,5]

    K.clear_session()

    input_layer = Input(shape=INPUT_SHAPE)
    x = ResContextBlock(32)(input_layer)
    x = ResContextBlock(32)(x)
    x = ResContextBlock(32)(x)
    x,resBlock1 = ResBlock(64, 0.2, pooling=True, drop_out=False)(x)

    x,resBlock2 = ResBlock(128, 0.2, pooling=True)(x)
    x,resBlock3 = ResBlock(128, 0.2, pooling=True)(x)
    x,resBlock4 = ResBlock(256, 0.2, pooling=True)(x)

    x = ResBlock(256, 0.2, pooling=False)(x)

    x = UpBlock(128, 0.2)(x,resBlock4)
    x = UpBlock(128, 0.2)(x,resBlock3)
    x = UpBlock(64, 0.2)(x,resBlock2)
    x = UpBlock(32, 0.2, drop_out=False)(x,resBlock1)

    logits = Conv2D(output_layers, kernel_size=(1, 1), activation="softmax")(x)

    model = Model(inputs=input_layer, outputs=logits)
    #model.summary()
    return model

