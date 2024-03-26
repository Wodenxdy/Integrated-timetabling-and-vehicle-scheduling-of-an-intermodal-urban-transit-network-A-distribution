# TTRDCP
This is the Gurobi code for TTRDCP

## Setup

### Prerequisites
- Python 3.x
- Gurobi Optimizer

### Installation

1. Ensure Gurobi and Python are installed and configured on your system.
2. Download the scripts to your local machine.

## Files

- `GenerateInstance.py`: Reads data and sets parameters for the model.
- `Gurobi_DRT_Group.py`: Generates the optimization model.
- `APP.py`: Main script that generates the model and solves it using Gurobi.

## Usage

Run `APP.py` to execute the model generation and optimization process:

## Data
- Four stations, two trips.
- 'passenger_demand_generate.npy': Randomly generated passenger flow data.
- 'running_nominal.npy': Randomly generated travel time.

```bash
python APP.py
