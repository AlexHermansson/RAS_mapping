def read_measurements(path_to_measurements, new_path):
    with open(path_to_measurements) as f:
        with open(new_path, "w+") as fnew:
            lines = f.readlines()
            for line in lines:
                coords = [float(c) for c in line.split(')')[0][1:].split(',')]
                ranges = [float(r) for r in line.split(')')[1][3:].split(',')]
                coords_str = ("%s" % coords)[1:-1] + '\n'
                ranges_str = ("%s" % ranges)[1:-1] + '\n'
                fnew.write(coords_str)
                fnew.write(ranges_str)


path_to_measurements = "measurements_new.txt"
new_path = "updated_measurements.txt"
read_measurements(path_to_measurements, new_path)