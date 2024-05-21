import torch
import cv2
import numpy as np

def preprocess_image(image):
    # Resize the image to the input size expected by the model
    image_resized = cv2.resize(image, (1280, 720))
    # Normalize the image
    image_normalized = image_resized / 255.0
    # Convert to tensor and add batch dimension
    image_tensor = torch.tensor(image_normalized, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0)
    return image_tensor

def postprocess_output(output):
    # Apply a threshold to the output to get a binary mask
    output_np = output.squeeze().detach().cpu().numpy()
    segmented_mask = (output_np > 0.5).astype(np.uint8)
    return segmented_mask

def evaluate(model, image):
    # Preprocess the input image
    image_tensor = preprocess_image(image)
    
    # Perform segmentation
    with torch.no_grad():
        model.eval()
        output = model(image_tensor)
    
    # Postprocess the output
    segmented_mask = postprocess_output(output)
    
    # Return the segmented lanes
    return segmented_mask
