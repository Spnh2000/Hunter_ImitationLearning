import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader, Subset
from sklearn.model_selection import KFold
import matplotlib.pyplot as plt
import numpy as np
import itertools

# ---------------------------------------------------------
# 1. Device Configuration
# ---------------------------------------------------------
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"Training on device: {device}")

# ---------------------------------------------------------
# 2. Data Preparation & Normalization
# ---------------------------------------------------------
data_path = '/content/drive/MyDrive/RPTU/Lab_project/offline_rl_dataset.npz'
data = np.load(data_path)

X = data['observations']  
Y = data['actions']       

# Normalize and clean LiDAR data
X = np.nan_to_num(X, nan=30.0, posinf=30.0, neginf=0.0)
X = X / 30.0  

# Normalize Actions (Y)
y_mean = np.mean(Y, axis=0)
y_std = np.std(Y, axis=0)
Y = (Y - y_mean) / (y_std + 1e-8)

print(f"Action Normalization Stats:")
print(f"Mean: {y_mean}")
print(f"Std: {y_std}")

stats_save_path = '/content/drive/MyDrive/RPTU/Lab_project/action_stats.npz'
np.savez(stats_save_path, mean=y_mean, std=y_std)
print(f"Saved action normalization stats to {stats_save_path}\n")

# Convert to Full Tensors (No split yet, KFold will handle it)
X_tensor = torch.tensor(X, dtype=torch.float32)
Y_tensor = torch.tensor(Y, dtype=torch.float32)
full_dataset = TensorDataset(X_tensor, Y_tensor)

# ---------------------------------------------------------
# 3. Dynamic Model Definition
# ---------------------------------------------------------
class ImitationPolicy(nn.Module):
    def __init__(self, cnn_channels, fc_hidden_sizes, kernel_size=5, dropout_rate=0.0, use_batch_norm=False):
        super(ImitationPolicy, self).__init__()
        
        # 1. Build 1D CNN feature extractor
        cnn_layers = []
        in_channels = 1
        
        for out_channels in cnn_channels:
            cnn_layers.append(nn.Conv1d(in_channels, out_channels, kernel_size=kernel_size, padding=kernel_size//2))
            if use_batch_norm:
                cnn_layers.append(nn.BatchNorm1d(out_channels))
            cnn_layers.append(nn.ReLU())
            cnn_layers.append(nn.MaxPool1d(kernel_size=2))
            in_channels = out_channels
            
        self.cnn = nn.Sequential(*cnn_layers)
        
        # Calculate flattened size
        length = 1080
        for _ in cnn_channels:
            length = length // 2
            
        flattened_size = cnn_channels[-1] * length if cnn_channels else 1080
        
        # 2. Build Fully Connected head
        fc_layers = []
        in_features = flattened_size
        
        for h in fc_hidden_sizes:
            fc_layers.append(nn.Linear(in_features, h))
            if use_batch_norm:
                fc_layers.append(nn.BatchNorm1d(h))
            fc_layers.append(nn.ReLU())
            if dropout_rate > 0.0:
                fc_layers.append(nn.Dropout(dropout_rate))
            in_features = h
            
        fc_layers.append(nn.Linear(in_features, 2)) # Output: [steering, speed]
        self.fc = nn.Sequential(*fc_layers)

    def forward(self, x):
        # x shape: [batch, 1080] -> Reshape for CNN: [batch, 1 channel, 1080]
        x = x.unsqueeze(1)
        x = self.cnn(x)
        x = x.view(x.size(0), -1) # Flatten
        return self.fc(x)

# ---------------------------------------------------------
# 4. Hyperparameter Grid Setup
# ---------------------------------------------------------
param_grid = {
    'lr': [1e-3],                                 # Learning Rates
    'batch_size': [128],                          # Batch Sizes
    'cnn_channels': [
        [16, 32, 64],
        [32, 64, 128],
        [16, 32, 64, 128]
    ], # CNN Depths and Widths
    'fc_hidden_sizes': [
        [256, 128],
        [512, 256, 128]
    ], # Fully Connected head capacity
    'kernel_size': [3, 5],                        # Receptive field size
    'dropout_rate': [0.0, 0.2],                   # Dropout probabilities
    'use_batch_norm': [True, False]               # Batch Normalization
}

# Generate all combinations
keys, values = zip(*param_grid.items())
combinations = [dict(zip(keys, v)) for v in itertools.product(*values)]

k_folds = 3
num_epochs = 25 # Reduced epochs for CV search to save time
kf = KFold(n_splits=k_folds, shuffle=True, random_state=42)

best_score = float('inf')
best_params = None

print(f"Starting {k_folds}-Fold Cross Validation across {len(combinations)} hyperparameter combinations...\n")

# ---------------------------------------------------------
# 5. Cross-Validation Loop
# ---------------------------------------------------------
for i, params in enumerate(combinations):
    print(f"--- Testing Combo {i+1}/{len(combinations)}: {params} ---")
    fold_val_losses = []
    
    for fold, (train_idx, val_idx) in enumerate(kf.split(full_dataset)):
        # Create Subsets for this fold
        train_sub = Subset(full_dataset, train_idx)
        val_sub = Subset(full_dataset, val_idx)
        
        train_loader = DataLoader(train_sub, batch_size=params['batch_size'], shuffle=True)
        val_loader = DataLoader(val_sub, batch_size=params['batch_size'], shuffle=False)
        
        # Initialize fresh model and optimizer
        model = ImitationPolicy(
            cnn_channels=params['cnn_channels'],
            fc_hidden_sizes=params['fc_hidden_sizes'],
            kernel_size=params['kernel_size'],
            dropout_rate=params['dropout_rate'],
            use_batch_norm=params['use_batch_norm']
        ).to(device)
        criterion = nn.MSELoss()
        optimizer = optim.Adam(model.parameters(), lr=params['lr'])
        
        # Train
        for epoch in range(num_epochs):
            model.train()
            for batch_x, batch_y in train_loader:
                batch_x, batch_y = batch_x.to(device), batch_y.to(device)
                optimizer.zero_grad()
                loss = criterion(model(batch_x), batch_y)
                loss.backward()
                optimizer.step()
                
        # Validate
        model.eval()
        running_val_loss = 0.0
        with torch.no_grad():
            for batch_x, batch_y in val_loader:
                batch_x, batch_y = batch_x.to(device), batch_y.to(device)
                loss = criterion(model(batch_x), batch_y)
                running_val_loss += loss.item() * batch_x.size(0)
                
        fold_loss = running_val_loss / len(val_sub)
        fold_val_losses.append(fold_loss)
        
    avg_val_loss = np.mean(fold_val_losses)
    print(f">>> Average Validation Loss: {avg_val_loss:.6f}\n")
    
    if avg_val_loss < best_score:
        best_score = avg_val_loss
        best_params = params

print(f"==================================================")
print(f"🏆 BEST HYPERPARAMETERS FOUND:")
print(f"{best_params} (Loss: {best_score:.6f})")
print(f"==================================================\n")

# ---------------------------------------------------------
# 6. Final Champion Model Training (50 Epochs)
# ---------------------------------------------------------
print("Training final champion model on 80/20 split using best hyperparameters...")

# Standard 80/20 split on shuffled data
indices = np.arange(len(X))
np.random.shuffle(indices)

split_idx = int(len(X) * 0.8)
train_dataset = Subset(full_dataset, indices[:split_idx])
val_dataset = Subset(full_dataset, indices[split_idx:])

train_loader = DataLoader(train_dataset, batch_size=best_params['batch_size'], shuffle=True)
val_loader = DataLoader(val_dataset, batch_size=best_params['batch_size'], shuffle=False)

champ_model = ImitationPolicy(
    cnn_channels=best_params['cnn_channels'],
    fc_hidden_sizes=best_params['fc_hidden_sizes'],
    kernel_size=best_params['kernel_size'],
    dropout_rate=best_params['dropout_rate'],
    use_batch_norm=best_params['use_batch_norm']
).to(device)
criterion = nn.MSELoss()
optimizer = optim.Adam(champ_model.parameters(), lr=best_params['lr'])

champ_train_losses, champ_val_losses = [], []
final_epochs = 50

for epoch in range(final_epochs):
    champ_model.train()
    running_train_loss = 0.0
    for batch_x, batch_y in train_loader:
        batch_x, batch_y = batch_x.to(device), batch_y.to(device)
        optimizer.zero_grad()
        loss = criterion(champ_model(batch_x), batch_y)
        loss.backward()
        optimizer.step()
        running_train_loss += loss.item() * batch_x.size(0)
        
    epoch_train_loss = running_train_loss / len(train_dataset)
    champ_train_losses.append(epoch_train_loss)
    
    champ_model.eval()
    running_val_loss = 0.0
    with torch.no_grad():
        for batch_x, batch_y in val_loader:
            batch_x, batch_y = batch_x.to(device), batch_y.to(device)
            loss = criterion(champ_model(batch_x), batch_y)
            running_val_loss += loss.item() * batch_x.size(0)
            
    epoch_val_loss = running_val_loss / len(val_dataset)
    champ_val_losses.append(epoch_val_loss)

# Plot final training curve
plt.figure(figsize=(8, 5))
plt.plot(champ_train_losses, label='Training Loss')
plt.plot(champ_val_losses, label='Validation Loss')
plt.xlabel('Epochs')
plt.ylabel('Mean Squared Error')
plt.title(f'Champion Model Training Curve')
plt.legend()
plt.grid(True)
plt.show()

# Save best model and metadata
save_path = '/content/drive/MyDrive/RPTU/Lab_project/imitation_policy.pth'
checkpoint = {
    'state_dict': champ_model.state_dict(),
    'hyperparameters': best_params,
    'action_mean': y_mean,
    'action_std': y_std
}
torch.save(checkpoint, save_path)
print(f"Champion Model saved successfully to: {save_path}")
