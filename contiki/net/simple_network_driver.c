#include "simple_network_driver.h"
#include "net/netstack.h"
#include "dev/leds.h"

static void (*m_callback)();


static void init(void) {

}

static void input(void) {
	(*m_callback)();
}

void simple_network_set_callback(void *callback) {
	m_callback = callback;
}

const struct network_driver simple_network_driver = {
	"Simple",
	init,
	input
};
