package main

// #include <time.h>
import "C"
import "fmt"
//import "time"
import "net"
import "bufio"
import "strings"
import "strconv"
import "os"

func main() {
	var tr2 C.struct_timespec
	var tr3 C.struct_timespec

	one_billion := (uint64)(1000000000)

	if (len(os.Args) != 2) {
		fmt.Printf("\n Usage: client <ip of server> \n");
		os.Exit(1)
	}


	// Conect to server
	ipv6addr := "["+os.Args[1]+"]:8081"
	conn, _ := net.Dial("tcp", ipv6addr)

	// Receive time from server
	// Do not care about message at this point
	message, _ := bufio.NewReader(conn).ReadString('\n')
//	fmt.Print("Server Time= " +message)

	// Get current time
	C.clock_gettime(C.CLOCK_REALTIME, &tr2)
	C.clock_gettime(C.CLOCK_REALTIME, &tr3)


	// Calc time difference
	secDiff := (uint64)(tr3.tv_sec)
	nsecDiff := (uint64)(tr3.tv_nsec)
	if nsecDiff < (uint64)(tr2.tv_nsec) {
		secDiff -= 1
		nsecDiff += one_billion
	}
	secDiff -= (uint64)(tr2.tv_sec)
	nsecDiff -= (uint64)(tr2.tv_nsec)


	sendBuff := fmt.Sprintf("%d:%d\n", (uint64)(secDiff), (uint64)(nsecDiff))
	fmt.Printf(sendBuff)
	conn.Write([]byte(sendBuff))

//	// Send client time to server
//	fmt.Fprintf(conn, buffer)

	// Receive updated time from server
	message, _ = bufio.NewReader(conn).ReadString('\n')
	fmt.Printf(message +"\n")


	msg := strings.Replace(message, "\n", "", -1)
	s_nums := strings.Split(msg, ":")
	tmpNum, _ := strconv.ParseUint(s_nums[0], 10, 64)
	s_sec := (uint64)( tmpNum )
	tmpNum, _ = strconv.ParseUint(s_nums[1], 10, 64)
	s_nsec := (uint64)( tmpNum )
	tmpNum, _ = strconv.ParseUint(s_nums[2], 10, 64)
	nsecDelay := (uint64)( tmpNum )
	fmt.Printf("s_nums[0]: %s\n", s_nums[0])
	fmt.Printf("s_nums[1]: %s\n", s_nums[1])
	fmt.Printf("s_nums[2]: %s\n", s_nums[2])

	s_nsec += nsecDelay
	for s_nsec > one_billion {
		s_sec += 1
		s_nsec -= one_billion
	}

	tr2.tv_sec = C.__time_t(s_sec)
	tr2.tv_nsec = C.__syscall_slong_t(s_nsec - nsecDelay)


	C.clock_settime(C.CLOCK_REALTIME, &tr2 )

	fmt.Printf("Time difference in ns: %d\n", (uint64)(nsecDelay))


	// close connection
	conn.Close()

}
