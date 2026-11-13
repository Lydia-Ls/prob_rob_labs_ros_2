# analyze_variance.py
import csv
import statistics
import json

def load_errors(filename, error_column, low, high):
    with open(filename) as f:
        rows = list(csv.DictReader(f))
    return [
        float(row[error_column])
        for row in rows
        if low <= float(row['measured_distance']) < high
    ]


# distance ranges: near, middle, far
ranges = [(1, 4), (4, 6.5), (6.5, 10.01)]
model = {}

for low, high in ranges:
    distance_errors = load_errors('distance_error.csv', 'distance_error', low, high)
    bearing_errors = load_errors('bearing_error.csv', 'bearing_error', low, high)
    if len(distance_errors) < 2 or len(bearing_errors) < 2:
        print(f'{low}-{high} m: insufficient data')
        continue

    d_var = statistics.variance(distance_errors)
    b_var = statistics.variance(bearing_errors)

    model[(low, high)] = (d_var, b_var)

    print(f'{low}-{high} m:')
    print(f'  Distance variance = {d_var:.8f} m²')
    print(f'  Bearing variance  = {b_var:.8f} rad²')
    with open('variance_model.json', 'w') as f:
        json.dump({f'{low}-{high}': [d_var, b_var] for (low, high), (d_var, b_var) in model.items()}, f)

def get_R(distance):
    for (low, high), (d_var, b_var) in model.items():
        if low <= distance < high:
            return [[d_var, 0], [0, b_var]]
    return None

# example
print('R(4.5 m) =', get_R(4.5))
