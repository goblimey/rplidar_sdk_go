package utils

import (
	"fmt"

	"github.com/goblimey/rplidar_sdk_go/driver"
)

func PrintNodes(nodes []driver.RplidarResponseMeasurementNode) {
	for i, node := range nodes {
		syncBit := " "
		if node.SyncBit() {
			syncBit = "S"
		}
		checkBit := " "
		if node.CheckBit() {
			checkBit = "C"
		}
		fmt.Printf("%3d [%s%s] A [0x%04x %3d %2d %6.2f] D [0x%04x %4d %2d %08.2f] Q %3d\n",
			i, syncBit, checkBit, node.AngleAsInt(),
			node.AngleAsInt()>>6, node.AngleAsInt()&0x3f, node.AngleAsFloat32(),
			node.AngleAsInt(), node.DistanceQ2>>2, node.DistanceQ2&0x3, node.DistanceAsFloat32(),
			node.Quality())
	}
}

func PlotHistogram(nodes []driver.RplidarResponseMeasurementNode) {

	const BARCOUNT int = 75
	const MAXBARHEIGHT int = 20
	const ANGLESCALE float32 = 360.0 / float32(BARCOUNT)

	histogram := make([]float32, BARCOUNT)

	for i := 0; i < BARCOUNT; i++ {
		histogram[i] = 0.0
	}

	maxVal := float32(0.0)
	for i := 0; i < len(nodes); i++ {
		intDeg := int(nodes[i].AngleAsFloat32() / ANGLESCALE)
		if intDeg >= BARCOUNT {
			intDeg = 0
		}

		cachedd := histogram[intDeg]
		if cachedd == 0.0 {
			cachedd = nodes[i].DistanceAsFloat32()
		} else {
			cachedd = (nodes[i].DistanceAsFloat32() + cachedd) / 2.0
		}

		if cachedd > maxVal {
			maxVal = cachedd
		}
		histogram[intDeg] = cachedd
	}

	for height := 0; height < MAXBARHEIGHT; height++ {
		thresholdH := float32(MAXBARHEIGHT-height-1) * (maxVal / float32(MAXBARHEIGHT))
		for xpos := 0; xpos < BARCOUNT; xpos++ {
			if histogram[xpos] >= thresholdH {
				fmt.Printf("*")
			} else {
				fmt.Printf(" ")
			}
		}
		fmt.Printf("\n")
	}
	for xpos := 0; xpos < BARCOUNT; xpos++ {
		fmt.Printf("-")
	}
	fmt.Printf("\n")
}
