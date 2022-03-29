#!/usr/bin/env python3

""" Script generates histogram plots of user specified signals.

    Example: Assume we have a signal called 'iteration' in several silo files.
             The script combines all silo files from an input directory, and
             calculates a histogram. The histogram is saved as a .png to the
             output folder.
"""

from typing import Dict

import argparse
import matplotlib.pyplot as plt
import numpy as np
import pathlib
import signal_logger

# Edit this list to process additional/different signals.
signals_for_evaluation = ['task_durations_ms/iteration',
                          'task_durations_ms/update',
                          'task_durations_ms/receive',
                          'task_durations_ms/advance',
                          'task_durations_ms/send']


def parse_arguments() -> argparse.Namespace:
    '''
    Wrapper function for argument parser.

    Returns:
        parse_args(argparse.Namespace):The parsed arguments.
    '''
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "silos_directory",
        help="Path to directory of silo files.")
    parser.add_argument(
        "output_directory",
        help="Path to output directory (i.e. for histogram plots).")
    parser.add_argument(
        "file_prefix",
        help="File prefix of silos (i.e. stateEstimator or motionControl).")
    return parser.parse_args()


def validate_directories(input_directory: pathlib.Path,
                         output_directory: pathlib.Path
                         ) -> bool:
    '''
    Checks the existence of the input directory

    Parameters:
        input_directory (pathlib.Path): Directory where silos are stored.
        output_directory (pathlib.Path): Output directory.

    Returns:
        (bool): Returns False if there is an error, otherwise True.
    '''
    if not input_directory.exists():
        print(
            f"Sorry '{input_directory}' does not exist. Exiting program.")
        return False

    if not output_directory.exists():
        print(
            f"Directory '{output_directory}' non existent. Creating it. \n")
        output_directory.mkdir(parents=True, exist_ok=True)

    return True


def create_dict_from_silos(silos_directory) -> Dict:
    '''
    Fills a dict with numpy arrays from silos in silos_directory
    from selected signals.

    Parameters:
        silos_directory (pathlib.Path): Directory where silos are stored.

    Returns:
        collection(Dict): Returns a dictionary with accumulated signals.
    '''
    collection = {}
    for silo_path in silos_directory:
        if silo_path.is_file():
            silo = signal_logger.Silo(silo_path, print_log_file_path=True)
            for signal in signals_for_evaluation:
                if signal in collection:
                    collection[signal] = np.concatenate(
                        (collection[signal], silo[signal].values))
                else:
                    collection[signal] = silo[signal].values
    return collection


def create_and_save_histograms_to_file(collection: Dict,
                                       output_directory: str,
                                       file_prefix: str
                                       ) -> None:
    '''
    Iterates through a dict and creates histogram plots out of its elements.
    File path format:
        output_directory/file_prefix_<signal_key>.png

    Parameters:
        collection(Dict): Input dictionary.
        output_directory(str): Output directory.
        file_prefix(str): File prefix for histogram plots.
    '''
    for key in collection:
        n, bins, patches = plt.hist(x=collection[key],
                                    bins='auto',
                                    color='#0504aa',
                                    alpha=0.7,
                                    rwidth=0.85)
        plt.grid(axis='y', alpha=0.75)
        plt.xlabel('ms')
        plt.ylabel('Frequency')
        plt.title(f'{key} ({collection[key].size} samples)')
        plt.text(23, 45, r'$\mu=15, b=3$')
        maxfreq = n.max()
        # Set a clean upper y-axis limit.
        plt.ylim(ymax=np.ceil(maxfreq / 10) * 10 if maxfreq %
                 10 else maxfreq + 10)

        filename = f'{file_prefix}_{key}.png'
        filename = filename.replace('/', '_')
        filepath = f'{output_directory}/{filename}'
        plt.savefig(filepath)
        plt.close()


if __name__ == '__main__':
    # Example usage:
    # python3 histogram.py /tmp/silo_folder /tmp/histograms stateEstimator
    args = parse_arguments()

    silos_directory = pathlib.Path(args.silos_directory)
    output_directory = pathlib.Path(args.output_directory)
    file_prefix = args.file_prefix

    if not validate_directories(silos_directory, output_directory):
        raise SystemExit(0)

    print(f"Input directory set to {silos_directory}")
    print(f"Output directory set to {output_directory} \n")

    print("Creating histogram of the following signals:")
    [print(f"- {signal}") for signal in signals_for_evaluation]

    print("\n === Starting to process silo files ===")

    filtered_silos_directory = silos_directory.glob(f'{file_prefix}*.silo')

    collection = create_dict_from_silos(filtered_silos_directory)

    create_and_save_histograms_to_file(collection,
                                       output_directory,
                                       file_prefix)
