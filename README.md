# Integrated timetabling and vehicle scheduling model
This is the Gurobi code for TTVSP model proposed in the paper

## Setup

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

## Paper
- Title: Integrated timetabling and vehicle scheduling of an intermodal urban transit network: A distributionally robust optimization approach
- Author: Dongyang Xia, Jihui Ma, Sh. Sharif Azadeh
- Abstract: Integrating emerging shared mobility with traditional fixed-line public transport is a promising solution to the mismatch between supply and demand in urban transportation systems. The advent of modular vehicles (MVs) provides opportunities for more flexible and seamless intermodal transit. The MVs, which have been implemented, are comprised of automated modular units (MUs), and can dynamically change the number of MUs comprising them at different times and stops. However, this innovative intermodal urban transit brings with it a new level of dynamism and uncertainty. In this paper, we study the problem of jointly optimizing the timetable and the vehicle schedule within an intermodal urban transit network utilizing MVs within the context of distributionally robust optimization (DRO), which allows MVs to dynamically (de)couple at each stop and permits flexible circulations of MUs across different transportation modes. We propose a DRO formulation to explore the trade-off between operators and passengers, with the objective of minimizing the worst-case expectation of the weighted sum of passengers’ and operating costs. Furthermore, to address the computational intractability of the proposed DRO model, we design a discrepancy-based ambiguity set to reformulate it into a mixed-integer linear programming model. In order to obtain high-quality solutionss of realistic instances, we develop a customized decomposition-based algorithm. Extensive numerical experiments demonstrate the effectiveness of the proposed approach. The computational results of real-world case studies based on the operational data of Beijing Bus Line illustrate that the proposed integrated timetabling and vehicle scheduling method reduces the expected value of passengers’ and operating costs by about 6% in comparison with the practical timetable and fixed-capacity vehicles typically used in the Beijing bus system.
- Doi:https://doi.org/10.1016/j.trc.2024.104610

## Flowchart for the hybride algorithm in the paper
<img width="786" alt="image" src="https://github.com/user-attachments/assets/7eaf3e73-14a8-4e7f-89ff-ca214780e5a6">

## Results
<img width="626" alt="image" src="https://github.com/user-attachments/assets/ee9e252e-0e4c-4365-bf17-29265f590a0a">


```bash
python APP.py



