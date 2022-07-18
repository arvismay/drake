for ts in 0.01 0.001 0.005;
do
	for x in 2 3;
	do
		for y in 2 3;
		do
			for z in 2 3 4 5;
			do
				bazel run //examples/boxpile:run_box_pile -- --resolution_hint_factor=2 --hydroelastic_modulus=300000 --gx=$x --gy=$y --gz=$z --mbp_dt=$ts --contact_model=hydroelastic
			done;
		done;
	done;
done
