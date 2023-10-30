//#include "usart.h"

/* MODIFIED VERSION OF ECE331 CODE */


// Transmit printf string to USART - blocking call
int _write(int file, void *ptr, size_t len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, 10000);
    return len;
}

// Called by printf, calls function to write to UAURT
_ssize_t _write_r(struct _reent *ptr, int fd, const void *buf, size_t cnt)
{
    return _write(fd, (char *) buf, cnt);
}

// Read in characters from UART using interrupt triggering
_ssize_t _read_r(struct _reent *ptr, int fd, void *buf, size_t cnt)
{
    if (cnt == 0)
	return 0;
    HAL_UART_Receive_IT(&huart2, (void *) buf, 1);	//receive one character
    return 1;
}
