import csv
import matplotlib.pyplot as plt

# ---- Step Response ----
step_samples = []
step_inputs = []
step_outputs = []

with open('./step_response.csv', 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        step_samples.append(int(row['Sample']))
        step_inputs.append(int(row['Input']))
        step_outputs.append(int(row['Output']))

# ---- Chirp Response ----
chirp_samples = []
chirp_inputs = []
chirp_outputs = []

with open('./chirp_response.csv', 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        chirp_samples.append(int(row['Sample']))
        chirp_inputs.append(int(row['Input']))
        chirp_outputs.append(int(row['Output']))

# ---- Plotting ----
plt.figure(figsize=(12, 10))

# Step Response Plot
plt.subplot(2, 1, 1)
plt.plot(step_samples, step_inputs, label='Input', linestyle='--')
plt.plot(step_samples, step_outputs, label='Output', marker='o', linestyle='-')
plt.title('Step Response')
plt.xlabel('Sample')
plt.ylabel('Amplitude')
plt.grid(True)
plt.legend()

# Chirp Response Plot
plt.subplot(2, 1, 2)
plt.plot(chirp_samples, chirp_inputs, label='Input', linestyle='--')
plt.plot(chirp_samples, chirp_outputs, label='Output', marker='o', linestyle='-')
plt.title('Chirp Response')
plt.xlabel('Sample')
plt.ylabel('Amplitude')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.savefig("./filter_responses_10k.png", dpi=300)
plt.show()

