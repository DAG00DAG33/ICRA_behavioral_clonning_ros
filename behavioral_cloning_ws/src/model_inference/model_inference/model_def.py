import torch
import numpy as np

import torch.nn as nn
import torch.nn.functional as F

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.pipe = nn.Sequential(
            nn.Dropout(p=0.3),
            nn.Conv1d(1, 5, 40, stride = 5),
            nn.ReLU(),
            nn.Conv1d(5, 5, 20, stride = 3),
            nn.ReLU(),
            nn.Conv1d(5, 7, 15, stride = 2),
            nn.ReLU(),
            nn.Flatten(),
            nn.Dropout(p=0.1),
            nn.Linear(175, 2),
            nn.Sigmoid(),
            #nn.Softmax(dim=1)
        )

    def forward(self, x):
        return self.pipe(x)
