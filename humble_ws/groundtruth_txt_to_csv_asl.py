#!/usr/bin/env python3
import argparse
import csv

def txt_to_asl(input_file, output_file, has_quat=True):
    with open(input_file, "r") as f_in, open(output_file, "w", newline="") as f_out:
        writer = csv.writer(f_out)

        # Write ASL header (now in ns)
        writer.writerow([
            "#timestamp[ns]",
            "p_RS_R_x [m]", "p_RS_R_y [m]", "p_RS_R_z [m]",
            "q_RS_w []", "q_RS_x []", "q_RS_y []", "q_RS_z []"
        ])

        for line in f_in:
            if not line.strip() or line.startswith("#"):
                continue  # skip empty or comment lines

            parts = line.strip().split()

            if has_quat:
                # Expect: timestamp x y z qx qy qz qw (timestamp in seconds)
                ts_s, x, y, z, qx, qy, qz, qw = parts
                ts_ns = str(int(round(float(ts_s) * 1e9)))
                writer.writerow([ts_ns, x, y, z, qw, qx, qy, qz])
            else:
                # Expect: timestamp x y z (timestamp in seconds)
                ts_s, x, y, z = parts
                ts_ns = str(int(round(float(ts_s) * 1e9)))
                writer.writerow([ts_ns, x, y, z, 1.0, 0.0, 0.0, 0.0])

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="Input TXT file")
    parser.add_argument("output", help="Output CSV file")
    parser.add_argument("--no-quat", action="store_true",
                        help="Input has only timestamp + xyz, insert identity quaternion")
    args = parser.parse_args()

    txt_to_asl(args.input, args.output, has_quat=not args.no_quat)
