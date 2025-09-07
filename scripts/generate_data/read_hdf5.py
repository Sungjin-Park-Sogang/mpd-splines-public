import h5py

with h5py.File("./data/env-robot/1757060639/dataset.hdf5", 'r') as f:
    key_list = list(f.keys())
    
    for key in key_list:
        data = f.get(key)[1:2]
        print(f"[{key}]: {data}")