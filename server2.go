package main

// #include <time.h>
// #include <stdlib.h>
import "C"
import "fmt"
//import "time"
import "net"
import "bufio"
import "strings"
import "strconv"



func main() {
	var tr C.struct_timespec
	var tr4 C.struct_timespec
	const num_avgs int = 10
	var delays [num_avgs]uint64

	var sum float64
	sum = 0.0

	one_billion := (uint64)(1000000000)

	fmt.Println("Launching server...")

	// listen on a interfaces
	ln, _ := net.Listen("tcp", ":8081")

	for {
		// accept connection on port
		fmt.Println("Waiting connection...")
		conn, _ := ln.Accept()
		
		sum = 0.0

		for i := 0; i < num_avgs; i++ {

			// Get current time
			C.clock_gettime(C.CLOCK_REALTIME, &tr)
	//		current_time := time.Now().Local()
	//		buffer := fmt.Sprintf("%s:%d\n", current_time.Format("02:Jan:2006:03:04"), tr.tv_nsec/1000)
			sendBuff := fmt.Sprintf("%d:%d\n", (uint64)(tr.tv_sec), (uint64)(tr.tv_nsec))
			fmt.Printf(sendBuff)
			
	
			// Send server time to client
			conn.Write([]byte(sendBuff))
	
			// Receive and print time from server
			message, _ := bufio.NewReader(conn).ReadString('\n')
	
			// Get current time into tr4
			C.clock_gettime(C.CLOCK_REALTIME, &tr4)
			sendBuff = fmt.Sprintf("%d:%d\n", (uint64)(tr4.tv_sec), (uint64)(tr4.tv_nsec))
			fmt.Printf(sendBuff)
	
			// Calc time difference
			// TODO: fix nsec subtraction problem with tr4.tv_nsec < tr.tv_nsec
			secDiff := (uint64)(tr4.tv_sec)
			nsecDiff := (uint64)(tr4.tv_nsec)
			if nsecDiff < (uint64)(tr.tv_nsec) {
				secDiff -= 1
				nsecDiff += one_billion
			}
			secDiff -= (uint64)(tr.tv_sec)
			nsecDiff -= (uint64)(tr.tv_nsec)
			fmt.Printf("secDiff/nsecDiff: %d/%d\n", (uint64)(secDiff), (uint64)(nsecDiff))
	
			// Convert total difference nsec
			nsecDiff = nsecDiff + (secDiff * one_billion)
	
			// Get client time difference
			msg := strings.Replace(message, "\n", "", -1)
			c_nums := strings.Split(msg, ":")
			tmpNum, _ := strconv.ParseUint(c_nums[0], 10, 64)
			c_secDiff := (uint64)( tmpNum )
			tmpNum, _ = strconv.ParseUint(c_nums[1], 10, 64)
			c_nsecDiff := (uint64)( tmpNum )
			c_nsecDiff = c_nsecDiff + (c_secDiff * one_billion)
			fmt.Printf("c_nums[0]: %s\n", c_nums[0])
			fmt.Printf("c_nums[1]: %s\n", c_nums[1])
			
	
			// Get the 1-way delay
			nsecDiff = nsecDiff - c_nsecDiff
			nsecDiff = nsecDiff / 2

			delays[i] = nsecDiff
			sum += (float64)(nsecDiff)
		}

		// Calc avg delay
		avg_nsec := (uint64)(sum / (float64)(num_avgs))


		// Send client current time to set
		C.clock_gettime(C.CLOCK_REALTIME, &tr)
		sendBuff := fmt.Sprintf("%d:%d:%d\n", (uint64)(tr.tv_sec), (uint64)(tr.tv_nsec), (uint64)(avg_nsec) )
		fmt.Printf(sendBuff)
		conn.Write([]byte(sendBuff))


	}


}
