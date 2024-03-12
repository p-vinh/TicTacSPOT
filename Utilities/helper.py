

# OPEN FILE
with open('requirements.txt', 'r') as file:
    # Go through each line in the file
    for line in file:
        # Replace white space between words with a single space
        line = '=='.join(line.split())
        # Write the line back to the file
        with open('requirements1.txt', 'a') as file:
            file.write(line + '\n')
    # Close the file
    file.close()
