/////////////////////////////////////////////////////////////////////////////
//
// File: rf_read_register.c
//
// Copyright S. Brennen Ball, 2010
//
// The author provides no guarantees, warantees, or promises, implied or
//  otherwise.  By using this software you agree to indemnify the author
//  of any damages incurred by using it.
//
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
/////////////////////////////////////////////////////////////////////////////

#include "rf.h"
#include "rf_src.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// unsigned char rf_read_rx_payload(unsigned char * dataptr, unsigned int len)
//
// Description:
//  Reads the next payload from the RX FIFO
//
// Parameters:
//  unsigned char * dataptr	- pointer to which the read data is stored
//  unsigned int len - number of bytes to read from the payload
//
// Return value:
//  Read value of the STATUS register
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char rf_read_rx_payload(unsigned char * dataptr, unsigned int len)
{
	unsigned char status;

	//Clear CE, read the payload, then set CE back
	rf_clear_ce();
	status = rf_spi_execute_command(RF_R_RX_PAYLOAD, dataptr, len, true);
	rf_set_ce();

	return status; //Return the STATUS register value
}
