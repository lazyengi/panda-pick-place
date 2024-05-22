from ultralytics import YOLO
import sys
sys.path.append('/home/giuseppe/.local/lib/python3.8/site-packages/pytorch')

# Load a model
# model = YOLO("yolov8n.yaml")  # build a new model from scratch
model = YOLO("/home/giuseppe/franka_ws/src/panda_vision_model/model/train3/weights/best.pt")  # load a pretrained model (recommended for training)

# Use the model
# model.train(data="coco128.yaml", epochs=3)  # train the model
# metrics = model.val()  # evaluate model performance on the validation set
results = model("/home/giuseppe/Downloads/v2/train/images", save=True,
              project="/home/giuseppe/Downloads/v2")  # predict on an image
# path = model.export(format="onnx")  # export the model to ONNX format