# Usage: awk -F ',' -f calcAverage.awk path/to/file/statistic.csv
# Example: awk -F ',' -f calcAverage.awk ../cases/X2000Y2000N800K8/statistic.csv
BEGIN {
	min = 10000
	max = 0
}
{
	for (i = 1; i <= NF; ++i)
	{
		sums[i] += $i
		if (i == NF-1)
		{
			if (min > $i) min = $i
			if (max < $i) max = $i
		}
	}
}
END {
	for (i = 1; i < NF; ++i)
		printf("%.1f, ", sums[i]/NR)
	printf("%f, %d ~ %d\n", 1000*sums[NF]/NR, min, max)
	delete sums
}
