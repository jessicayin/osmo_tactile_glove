import os

def get_extracted_filenames(extracted_data_path):
    """
    get all the filenames that end with `.pkl` in extracted_data_path

    ---
    extracted_data_path: location to search for the extracted sync data .pkl 
    """
    all_filenames = []
    for root, dirs, files in os.walk(extracted_data_path):
        for file in files:
            if file.endswith(".pkl"):
                all_filenames.append(file)
    all_filenames.sort()

    return all_filenames

def split_filenames_by_task(filenames):
    """
    Given a list of file names with the naming convention {task}_{###}.pkl, split them into lists organized by the task name

    Args:
        filenames (List): List of string filenames following format: {task}_{###}.pkl (i.e. ["pick_000.pkl", "place_000.pkl", "place_001.pkl", "grasp_000.pkl])

    Returns:
        (List): where the filenames have been organized by task name (i.e. [["pick_000.pkl"], ["place_000.pkl", "place_001.pkl"], ["grasp_000.pkl]])
    """
    grouped_filenames = {}
    for filename in filenames:
        # Split the filename at the last underscore
        prefix, _ = filename.rsplit("_", 1)

        # If the prefix is not already in the dictionary, add it with an empty list
        if prefix not in grouped_filenames:
            grouped_filenames[prefix] = []

        # Add the filename to the list for its prefix
        grouped_filenames[prefix].append(filename)
    separate_lists = list(grouped_filenames.values())
    # print(separate_lists)
    return separate_lists