import sys

with open(sys.argv[1], 'r') as f:
    lines = f.readlines()

new_lines = []
for line in lines:
    if line.strip().startswith('#') or not line.strip():
        new_lines.append(line)
        continue

    # Squash minor changes
    if 'minor changes' in line or 'minro chagnes 2' in line:
        new_lines.append(line.replace('pick ', 'squash ', 1))
    elif 'Move PCB and architecture diagrams' in line or 'Merge: Unified Kalman Filter' in line or 'Fix: Resolved Kalman Filter' in line:
        new_lines.append(line.replace('pick ', 'reword ', 1))
    else:
        new_lines.append(line)

with open(sys.argv[1], 'w') as f:
    f.writelines(new_lines)
