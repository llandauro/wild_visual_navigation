import torch
import os
from BaseWVN import WVN_ROOT_DIR
a=torch.tensor([1,2,3]).unsqueeze(0)
b=torch.tensor([4,5]).unsqueeze(0)

print(torch.cat((a,b),dim=1))

c="debug"
d="debug+label"
if "debug" in d:
    print("yes")
model_folder=WVN_ROOT_DIR+"/model"
manage_folder=WVN_ROOT_DIR+"/results/manager"
m=torch.load(model_folder+"/last_checkpoint.pt")
d=torch.load(manage_folder+"/graph_data.pt")

pass
