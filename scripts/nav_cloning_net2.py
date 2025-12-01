import numpy as np
import os
import time
import copy
import cv2
import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import transforms
from torch.utils.data import DataLoader, TensorDataset
from pytorch_grad_cam import EigenCAM
from pytorch_grad_cam.utils.image import show_cam_on_image
import roslib

# HYPER PARAMETER
BATCH_SIZE = 8

class Net(nn.Module):
    def __init__(self, n_channel, n_out):
        super().__init__()
        self.conv1 = nn.Conv2d(n_channel, 32, kernel_size=8, stride=4)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=2)
        self.conv3 = nn.Conv2d(64, 64, kernel_size=3, stride=1)
        self.fc4 = nn.Linear(960, 512)
        self.fc5 = nn.Linear(512, n_out)
        self.relu = nn.ReLU(inplace=True)
        self.flatten = nn.Flatten()
        self.cnn_layer = nn.Sequential(
            self.conv1, self.relu,
            self.conv2, self.relu,
            self.conv3, self.relu,
            self.flatten
        )
        self.fc_layer = nn.Sequential(
            self.fc4, self.relu,
            self.fc5,
        )

    def forward(self, x):
        x1 = self.cnn_layer(x)
        x2 = self.fc_layer(x1)
        return x2

class deep_learning:
    def __init__(self, n_channel=3, n_action=1):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.net = Net(n_channel, n_action).to(self.device)
        self.n_action = n_action
        self.count = 0
        self.cam_dir = roslib.packages.get_pkg_dir('nav_cloning') + "/eigen/normal_network_frames/" + time.strftime("%Y%m%d_%H:%M:%S")
        self.cam_video_dir = roslib.packages.get_pkg_dir('nav_cloning') + "/eigen/normal_network_videos/"+ time.strftime("%Y%m%d_%H:%M:%S")
        os.makedirs(self.cam_dir, exist_ok=True)
        os.makedirs(self.cam_video_dir, exist_ok=True)

    def visualize_cam(self, img, save_path=None):
        rgb_img = np.float32(img) / 255.0
        if rgb_img.shape[-1] == 3:
            input_tensor = torch.tensor(rgb_img, dtype=torch.float32, device=self.device).permute(2, 0, 1).unsqueeze(0)
        else:
            raise ValueError(f"Unexpected CAM input shape: {rgb_img.shape}")

        cam = EigenCAM(model=self.net, target_layers=[self.net.conv3])
        grayscale_cam = cam(input_tensor=input_tensor)[0]
        heatmap = cv2.applyColorMap(np.uint8(255 * grayscale_cam), cv2.COLORMAP_JET)
        blended = cv2.addWeighted(np.uint8(np.clip(img * 255.0, 0, 255)), 0.8, heatmap, 0.2, 0)
        resized = cv2.resize(blended, (640, 480))
        if save_path:
            cv2.imwrite(save_path, resized)


    def load(self, load_path):
        self.net.load_state_dict(torch.load(load_path, map_location=self.device))

    def visualize_cam(self, img, save_path=None):
        rgb_img = np.float32(img) / 255.0
        input_tensor = torch.tensor(rgb_img, dtype=torch.float32, device=self.device).unsqueeze(0).permute(0, 3, 1, 2)
        cam = EigenCAM(model=self.net, target_layers=[self.net.conv3])
        grayscale_cam = cam(input_tensor=input_tensor)[0]
        heatmap = cv2.applyColorMap(np.uint8(255 * grayscale_cam), cv2.COLORMAP_JET)
        blended = cv2.addWeighted(np.uint8(np.clip(img * 255.0, 0, 255)), 0.8, heatmap, 0.2, 0)
        resized = cv2.resize(blended, (640, 480))
        if save_path:
            cv2.imwrite(save_path, resized)

    def save_cam_video(self, fps=10):
        image_files = sorted([f for f in os.listdir(self.cam_dir) if f.endswith(".jpg")])
        if not image_files:
            print("No CAM frames to make video.")
            return
        frame = cv2.imread(os.path.join(self.cam_dir, image_files[0]))
        height, width, _ = frame.shape
        out = cv2.VideoWriter(self.cam_video_dir + "/eigen_cam_video_normal_network.mp4",
                              cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))
        for filename in image_files:
            out.write(cv2.imread(os.path.join(self.cam_dir, filename)))
        out.release()
        print(f"✅ Saved CAM video to {self.cam_video_dir}")

if __name__ == '__main__':
    dl = deep_learning()

