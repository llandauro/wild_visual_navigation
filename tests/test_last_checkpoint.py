import os
import torch

checkpoint = torch.load("../assets/checkpoints/last_checkpoint.pt", map_location=torch.device('cpu'))

print(f"Checkpoint's keys: {checkpoint.keys()}")

#model_state_dict = checkpoint.get('model_state_dict', checkpoint)
#print(f"Model state dict's keys: {model_state_dict.keys()}")

#model_state_dict = checkpoint['model_state_dict']
#print("Model state dict keys and and their prefixes:")
#for key in model_state_dict.keys():
#    print(key)

#check other components:

print("\nTraversability loss state dict keys:")
print(checkpoint['traversability_loss_state_dict'].keys())

#Extract traversability loss state dict:
traversability_loss_state_dict = checkpoint['traversability_loss_state_dict']

confidence_keys = ['_confidence_generator.mean', '_confidence_generator.var', '_confidence_generator.std']
for key in confidence_keys:
    if key in traversability_loss_state_dict:
        value = traversability_loss_state_dict[key]
        print(f"{key}: ")
        print(value) # print tensor
        print(f"Shape: {value.shape}") # print shape of tensor
        print(f"Is value empty? {'Yes' if value.nelement() == 0 else 'No'}\n")  # Check if tensor is empty
    else:
        print(f"Key '{key}' not found in traversability_loss_state_dict.")
