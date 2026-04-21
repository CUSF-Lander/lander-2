import sys

with open(sys.argv[1], 'r') as f:
    text = f.read()

new_text = text

if 'Move PCB and architecture diagrams' in text:
    new_text = new_text.replace('Move PCB and architecture diagrams to non_firmware', 'Move architecture diagrams to non_firmware')

if 'Merge: Unified Kalman Filter & Fragmentation-based GPS RTK' in text:
    new_text = new_text.replace('PCB/Diagram files', 'diagram files')
    new_text = new_text.replace('Decoupled PCB/Diagram files', 'Decoupled diagram files')

if 'Fix: Resolved Kalman Filter' in text or 'minro chagnes 2' in text:
    # This is a squash or reword of the fix. Replace the whole thing with a natural message.
    lines = []
    # Drop everything that is not a comment and just provide the new message
    for line in new_text.split('\n'):
        if line.startswith('#'):
            lines.append(line)
    
    clean_msg = "Resolve Kalman Filter stack overflow and ESP-NOW fragmentation buffer vulnerability\n\nIncluded minor bug fixes and whitespace adjustments."
    new_text = clean_msg + "\n\n" + "\n".join(lines)

with open(sys.argv[1], 'w') as f:
    f.write(new_text)
