import torch
from PIL import Image
from torchvision import transforms
import os



CLASS_NAMES = [
    'iqoo 10 pro', 'iqoo 8 pro', 'iqoo 9 pro', 'oppo findx2 pro', 
    'oppo findx3 pro', 'oppo findx5 pro', 'oppo findx6 pro', 
    'oppo reno6 pro', 'oppo reno7 pro', 'oppo reno8 pro', 
    'realme 11 pro', 'realme GT2 pro', 'realme GTneo3', 
    'realme GTneo5', 'realme Q3 pro'
]


def load_model(filepath):
    model = torch.load(filepath, map_location=torch.device("cpu"))
    model.eval()
    return model

def process_image(image):
    transform = transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225]
        )
    ])
    img_tensor = transform(image)
    img_tensor = img_tensor.unsqueeze(0)  # Add batch dimension
    return img_tensor

def predict(image, model):
    img_tensor = process_image(image)
    with torch.no_grad():
        outputs = model(img_tensor)
        _, predicted_index = torch.max(outputs, 1)
    return CLASS_NAMES[predicted_index.item()]
