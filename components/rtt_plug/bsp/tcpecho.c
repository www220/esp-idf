#include <rtthread.h>
#include <lwip/api.h>

#define TCP_ECHO_PORT   7

void tcpecho_entry(void *parameter)
{
	struct netconn *conn, *newconn;
	err_t err;

	/* Create a new connection identifier. */
	conn = netconn_new(NETCONN_TCP);

	/* Bind connection to well known port number 7. */
	netconn_bind(conn, NULL, TCP_ECHO_PORT);

	/* Tell connection to go into listening mode. */
	netconn_listen(conn);

	while(1)
	{
		/* Grab new connection. */
		err = netconn_accept(conn, &newconn);
		/* Process the new connection. */
		if(err == ERR_OK)
		{
			struct netbuf *buf;
			void *data;
			u16_t len;
			char deal = 1;

			while(deal && netconn_recv(newconn, &buf) == ERR_OK)
			{
				do
				{
					netbuf_data(buf, &data, &len);
					err = netconn_write(newconn, data, len, NETCONN_COPY);
					if(err != ERR_OK)
					{
						rt_kprintf("tcpecho send failed\n");
						deal = 0;
						break;
					}
				}
				while(netbuf_next(buf) >= 0);
				netbuf_delete(buf);
			}
			/* Close connection and discard connection identifier. */
			netconn_delete(newconn);
		}
	}
}

static rt_thread_t echo_tid = RT_NULL;
void tcpecho(void)
{
    if (echo_tid == RT_NULL)
    {
        echo_tid = rt_thread_create("echo",
			tcpecho_entry, RT_NULL,
            4096, 30, 20);
        if (echo_tid != RT_NULL)
            rt_thread_startup(echo_tid);
    }
    else
    {
        rt_thread_delete(echo_tid); /* delete thread */
        echo_tid = RT_NULL;
    }
}

#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(tcpecho, startup or stop TCP echo server);
MSH_CMD_EXPORT(tcpecho, startup or stop TCP echo server);
#endif
