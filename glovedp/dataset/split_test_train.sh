#!/bin/bash

# Configurable parameters
SOURCE_FOLDERS=("./data/0912_wipe_sea" "./data/0915_wipe_sea")
DEST_BASE="./data/1024_wipe_sea"
TEST_RATIO=0.1  # 10% for test, 90% for train
COPY_MODE="cp"  # Use "mv" to move files instead of copying

# Create destination directories
mkdir -p "${DEST_BASE}/test"
mkdir -p "${DEST_BASE}/train"

echo "Creating test/train split with ${TEST_RATIO} ratio for test set"
echo "Destination: ${DEST_BASE}/"

# Function to get all subdirectories and sort them numerically
get_sorted_subdirs() {
    local source_dir="$1"
    if [[ -d "$source_dir" ]]; then
        find "$source_dir" -mindepth 1 -maxdepth 1 -type d | sort -V
    fi
}

# Collect all subdirectories from all source folders
all_subdirs=()
for source_folder in "${SOURCE_FOLDERS[@]}"; do
    if [[ -d "$source_folder" ]]; then
        echo "Processing source folder: $source_folder"
        while IFS= read -r -d '' subdir; do
            all_subdirs+=("$subdir")
        done < <(get_sorted_subdirs "$source_folder" | tr '\n' '\0')
    else
        echo "Warning: Source folder '$source_folder' does not exist"
    fi
done

# Calculate split point
total_dirs=${#all_subdirs[@]}
test_count=$(echo "$total_dirs * $TEST_RATIO" | bc -l | cut -d. -f1)
train_count=$((total_dirs - test_count))

echo "Total directories: $total_dirs"
echo "Test set: $test_count directories"
echo "Train set: $train_count directories"

# Shuffle the array for random split (optional - remove if you want deterministic split)
# Uncomment the next line for random shuffling
# all_subdirs=($(printf '%s\n' "${all_subdirs[@]}" | shuf))

# Split into test and train
counter=0
for subdir in "${all_subdirs[@]}"; do
    # Extract the subdirectory name (e.g., "0000" from "0912_wipe_sea/0000")
    subdir_name=$(basename "$subdir")
    source_parent=$(basename "$(dirname "$subdir")")
    
    # Determine destination based on counter
    if [[ $counter -lt $test_count ]]; then
        dest_dir="${DEST_BASE}/test/${source_parent}_${subdir_name}"
        dataset_type="test"
    else
        dest_dir="${DEST_BASE}/train/${source_parent}_${subdir_name}"
        dataset_type="train"
    fi
    
    # Copy or move the directory
    echo "Moving $subdir -> $dest_dir ($dataset_type)"
    $COPY_MODE -r "$subdir" "$dest_dir"
    
    ((counter++))
done

echo "Split complete!"
echo "Test set: ${DEST_BASE}/test/ (${test_count} directories)"
echo "Train set: ${DEST_BASE}/train/ (${train_count} directories)"
