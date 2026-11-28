with open('/tmp/fix_udev.py', 'r') as f:
    code = f.read()

# Обновить правило udev — поставить 0666 явно
code = code.replace(
    'KERNEL=="ttyAMA0", GROUP="dialout", MODE="0666"',
    'KERNEL=="ttyAMA0", MODE="0666", GROUP="dialout"\nKERNEL=="ttyAMA0", OWNER="root", GROUP="dialout"'
)

with open('/tmp/fix_udev.py', 'w') as f:
    f.write(code)
print('UPDATED')
