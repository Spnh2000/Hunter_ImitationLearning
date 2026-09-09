# Source - https://stackoverflow.com/a/58131092
# Posted by KC PARK, modified by community. See post 'Timeline' for change history
# Retrieved 2026-09-09, License - CC BY-SA 4.0

import numpy as np
import sys
import os

def main():
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        # Default to the dataset in the same directory as this script
        filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'offline_rl_dataset.npz')

    print(f"Loading dataset from: {filepath}")
    try:
        data = np.load(filepath, allow_pickle=True)
    except FileNotFoundError:
        print(f"Error: Could not find file at {filepath}")
        return

    lst = data.files
    print(f"\nDataset contains {len(lst)} arrays:")
    for item in lst:
        array = data[item]
        print(f"\n--- {item} ---")
        print(f"Shape: {array.shape}")
        print(f"Dtype: {array.dtype}")
        if array.size > 0:
            print(f"Sample [0]: {array[0]}")

if __name__ == '__main__':
    main()

