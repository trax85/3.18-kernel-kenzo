/******************************************************************************
 *
 * Module Name: tbutils - ACPI Table utilities
 *
 *****************************************************************************/

/*
 * Copyright (C) 2000 - 2014, Intel Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    substantially similar to the "NO WARRANTY" disclaimer below
 *    ("Disclaimer") and any redistribution must be conditioned upon
 *    including a substantially similar Disclaimer requirement for further
 *    binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGES.
 */

#include <acpi/acpi.h>
#include "accommon.h"
#include "actables.h"

#define _COMPONENT          ACPI_TABLES
ACPI_MODULE_NAME("tbutils")

/* Local prototypes */
static acpi_physical_address
acpi_tb_get_root_table_entry(u8 *table_entry, u32 table_entry_size);

#if (!ACPI_REDUCED_HARDWARE)
/*******************************************************************************
 *
 * FUNCTION:    acpi_tb_initialize_facs
 *
 * PARAMETERS:  None
 *
 * RETURN:      Status
 *
 * DESCRIPTION: Create a permanent mapping for the FADT and save it in a global
 *              for accessing the Global Lock and Firmware Waking Vector
 *
 ******************************************************************************/

acpi_status acpi_tb_initialize_facs(void)
{
	acpi_status status;

	/* If Hardware Reduced flag is set, there is no FACS */

	if (acpi_gbl_reduced_hardware) {
		acpi_gbl_FACS = NULL;
		return (AE_OK);
	}

	status = acpi_get_table_by_index(ACPI_TABLE_INDEX_FACS,
					 ACPI_CAST_INDIRECT_PTR(struct
								acpi_table_header,
								&acpi_gbl_FACS));
	return (status);
}
#endif				/* !ACPI_REDUCED_HARDWARE */

/*******************************************************************************
 *
 * FUNCTION:    acpi_tb_tables_loaded
 *
 * PARAMETERS:  None
 *
 * RETURN:      TRUE if required ACPI tables are loaded
 *
 * DESCRIPTION: Determine if the minimum required ACPI tables are present
 *              (FADT, FACS, DSDT)
 *
 ******************************************************************************/

u8 acpi_tb_tables_loaded(void)
{

	if (acpi_gbl_root_table_list.current_table_count >= 3) {
		return (TRUE);
	}

	return (FALSE);
}

/*******************************************************************************
 *
<<<<<<< HEAD
=======
 * FUNCTION:    acpi_tb_fix_string
 *
 * PARAMETERS:  string              - String to be repaired
 *              length              - Maximum length
 *
 * RETURN:      None
 *
 * DESCRIPTION: Replace every non-printable or non-ascii byte in the string
 *              with a question mark '?'.
 *
 ******************************************************************************/

static void acpi_tb_fix_string(char *string, acpi_size length)
{

	while (length && *string) {
		if (!ACPI_IS_PRINT(*string)) {
			*string = '?';
		}
		string++;
		length--;
	}
}

/*******************************************************************************
 *
 * FUNCTION:    acpi_tb_cleanup_table_header
 *
 * PARAMETERS:  out_header          - Where the cleaned header is returned
 *              header              - Input ACPI table header
 *
 * RETURN:      Returns the cleaned header in out_header
 *
 * DESCRIPTION: Copy the table header and ensure that all "string" fields in
 *              the header consist of printable characters.
 *
 ******************************************************************************/

static void
acpi_tb_cleanup_table_header(struct acpi_table_header *out_header,
			     struct acpi_table_header *header)
{

	ACPI_MEMCPY(out_header, header, sizeof(struct acpi_table_header));

	acpi_tb_fix_string(out_header->signature, ACPI_NAME_SIZE);
	acpi_tb_fix_string(out_header->oem_id, ACPI_OEM_ID_SIZE);
	acpi_tb_fix_string(out_header->oem_table_id, ACPI_OEM_TABLE_ID_SIZE);
	acpi_tb_fix_string(out_header->asl_compiler_id, ACPI_NAME_SIZE);
}

/*******************************************************************************
 *
 * FUNCTION:    acpi_tb_print_table_header
 *
 * PARAMETERS:  address             - Table physical address
 *              header              - Table header
 *
 * RETURN:      None
 *
 * DESCRIPTION: Print an ACPI table header. Special cases for FACS and RSDP.
 *
 ******************************************************************************/

void
acpi_tb_print_table_header(acpi_physical_address address,
			   struct acpi_table_header *header)
{
	struct acpi_table_header local_header;

	if (ACPI_COMPARE_NAME(header->signature, ACPI_SIG_FACS)) {

		/* FACS only has signature and length fields */

		ACPI_INFO((AE_INFO, "%4.4s 0x%8.8X%8.8X %05X",
			   header->signature, ACPI_FORMAT_UINT64(address),
			   header->length));
	} else if (ACPI_COMPARE_NAME(header->signature, ACPI_SIG_RSDP)) {

		/* RSDP has no common fields */

		ACPI_MEMCPY(local_header.oem_id,
			    ACPI_CAST_PTR(struct acpi_table_rsdp,
					  header)->oem_id, ACPI_OEM_ID_SIZE);
		acpi_tb_fix_string(local_header.oem_id, ACPI_OEM_ID_SIZE);

		ACPI_INFO((AE_INFO, "RSDP 0x%8.8X%8.8X %05X (v%.2d %6.6s)",
			   ACPI_FORMAT_UINT64(address),
			   (ACPI_CAST_PTR(struct acpi_table_rsdp, header)->
			    revision >
			    0) ? ACPI_CAST_PTR(struct acpi_table_rsdp,
					       header)->length : 20,
			   ACPI_CAST_PTR(struct acpi_table_rsdp,
					 header)->revision,
			   local_header.oem_id));
	} else {
		/* Standard ACPI table with full common header */

		acpi_tb_cleanup_table_header(&local_header, header);

		ACPI_INFO((AE_INFO,
			   "%-4.4s 0x%8.8X%8.8X %05X (v%.2d %-6.6s %-8.8s %08X %-4.4s %08X)",
			   local_header.signature, ACPI_FORMAT_UINT64(address),
			   local_header.length, local_header.revision,
			   local_header.oem_id, local_header.oem_table_id,
			   local_header.oem_revision,
			   local_header.asl_compiler_id,
			   local_header.asl_compiler_revision));

	}
}

/*******************************************************************************
 *
 * FUNCTION:    acpi_tb_validate_checksum
 *
 * PARAMETERS:  table               - ACPI table to verify
 *              length              - Length of entire table
 *
 * RETURN:      Status
 *
 * DESCRIPTION: Verifies that the table checksums to zero. Optionally returns
 *              exception on bad checksum.
 *
 ******************************************************************************/

acpi_status acpi_tb_verify_checksum(struct acpi_table_header *table, u32 length)
{
	u8 checksum;

	/* Compute the checksum on the table */

	checksum = acpi_tb_checksum(ACPI_CAST_PTR(u8, table), length);

	/* Checksum ok? (should be zero) */

	if (checksum) {
		ACPI_BIOS_WARNING((AE_INFO,
				   "Incorrect checksum in table [%4.4s] - 0x%2.2X, "
				   "should be 0x%2.2X",
				   table->signature, table->checksum,
				   (u8)(table->checksum - checksum)));

#if (ACPI_CHECKSUM_ABORT)

		return (AE_BAD_CHECKSUM);
#endif
	}

	return (AE_OK);
}

/*******************************************************************************
 *
 * FUNCTION:    acpi_tb_checksum
 *
 * PARAMETERS:  buffer          - Pointer to memory region to be checked
 *              length          - Length of this memory region
 *
 * RETURN:      Checksum (u8)
 *
 * DESCRIPTION: Calculates circular checksum of memory region.
 *
 ******************************************************************************/

u8 acpi_tb_checksum(u8 *buffer, u32 length)
{
	u8 sum = 0;
	u8 *end = buffer + length;

	while (buffer < end) {
		sum = (u8) (sum + *(buffer++));
	}

	return (sum);
}

/*******************************************************************************
 *
>>>>>>> p9x
 * FUNCTION:    acpi_tb_check_dsdt_header
 *
 * PARAMETERS:  None
 *
 * RETURN:      None
 *
 * DESCRIPTION: Quick compare to check validity of the DSDT. This will detect
 *              if the DSDT has been replaced from outside the OS and/or if
 *              the DSDT header has been corrupted.
 *
 ******************************************************************************/

void acpi_tb_check_dsdt_header(void)
{

	/* Compare original length and checksum to current values */

	if (acpi_gbl_original_dsdt_header.length != acpi_gbl_DSDT->length ||
	    acpi_gbl_original_dsdt_header.checksum != acpi_gbl_DSDT->checksum) {
		ACPI_BIOS_ERROR((AE_INFO,
				 "The DSDT has been corrupted or replaced - "
				 "old, new headers below"));
		acpi_tb_print_table_header(0, &acpi_gbl_original_dsdt_header);
		acpi_tb_print_table_header(0, acpi_gbl_DSDT);

		ACPI_ERROR((AE_INFO,
			    "Please send DMI info to linux-acpi@vger.kernel.org\n"
			    "If system does not work as expected, please boot with acpi=copy_dsdt"));

		/* Disable further error messages */

		acpi_gbl_original_dsdt_header.length = acpi_gbl_DSDT->length;
		acpi_gbl_original_dsdt_header.checksum =
		    acpi_gbl_DSDT->checksum;
	}
}

/*******************************************************************************
 *
 * FUNCTION:    acpi_tb_copy_dsdt
 *
 * PARAMETERS:  table_desc          - Installed table to copy
 *
 * RETURN:      None
 *
 * DESCRIPTION: Implements a subsystem option to copy the DSDT to local memory.
 *              Some very bad BIOSs are known to either corrupt the DSDT or
 *              install a new, bad DSDT. This copy works around the problem.
 *
 ******************************************************************************/

struct acpi_table_header *acpi_tb_copy_dsdt(u32 table_index)
{
	struct acpi_table_header *new_table;
	struct acpi_table_desc *table_desc;

	table_desc = &acpi_gbl_root_table_list.tables[table_index];

	new_table = ACPI_ALLOCATE(table_desc->length);
	if (!new_table) {
		ACPI_ERROR((AE_INFO, "Could not copy DSDT of length 0x%X",
			    table_desc->length));
		return (NULL);
	}

	ACPI_MEMCPY(new_table, table_desc->pointer, table_desc->length);
	acpi_tb_uninstall_table(table_desc);

	acpi_tb_init_table_descriptor(&acpi_gbl_root_table_list.
				      tables[ACPI_TABLE_INDEX_DSDT],
				      ACPI_PTR_TO_PHYSADDR(new_table),
				      ACPI_TABLE_ORIGIN_INTERNAL_VIRTUAL,
				      new_table);

	ACPI_INFO((AE_INFO,
		   "Forced DSDT copy: length 0x%05X copied locally, original unmapped",
		   new_table->length));

	return (new_table);
}

/*******************************************************************************
 *
<<<<<<< HEAD
=======
 * FUNCTION:    acpi_tb_install_table
 *
 * PARAMETERS:  address                 - Physical address of DSDT or FACS
 *              signature               - Table signature, NULL if no need to
 *                                        match
 *              table_index             - Index into root table array
 *
 * RETURN:      None
 *
 * DESCRIPTION: Install an ACPI table into the global data structure. The
 *              table override mechanism is called to allow the host
 *              OS to replace any table before it is installed in the root
 *              table array.
 *
 ******************************************************************************/

void
acpi_tb_install_table(acpi_physical_address address,
		      char *signature, u32 table_index)
{
	struct acpi_table_header *table;
	struct acpi_table_header *final_table;
	struct acpi_table_desc *table_desc;

	if (!address) {
		ACPI_ERROR((AE_INFO,
			    "Null physical address for ACPI table [%s]",
			    signature));
		return;
	}

	/* Map just the table header */

	table = acpi_os_map_memory(address, sizeof(struct acpi_table_header));
	if (!table) {
		ACPI_ERROR((AE_INFO,
			    "Could not map memory for table [%s] at %8.8X%8.8X",
			    signature, ACPI_FORMAT_UINT64(address)));
		return;
	}

	/* If a particular signature is expected (DSDT/FACS), it must match */

	if (signature && !ACPI_COMPARE_NAME(table->signature, signature)) {
		ACPI_BIOS_ERROR((AE_INFO,
				 "Invalid signature 0x%X for ACPI table, expected [%s]",
				 *ACPI_CAST_PTR(u32, table->signature),
				 signature));
		goto unmap_and_exit;
	}

	/*
	 * Initialize the table entry. Set the pointer to NULL, since the
	 * table is not fully mapped at this time.
	 */
	table_desc = &acpi_gbl_root_table_list.tables[table_index];

	table_desc->address = address;
	table_desc->pointer = NULL;
	table_desc->length = table->length;
	table_desc->flags = ACPI_TABLE_ORIGIN_MAPPED;
	ACPI_MOVE_32_TO_32(table_desc->signature.ascii, table->signature);

	/*
	 * ACPI Table Override:
	 *
	 * Before we install the table, let the host OS override it with a new
	 * one if desired. Any table within the RSDT/XSDT can be replaced,
	 * including the DSDT which is pointed to by the FADT.
	 *
	 * NOTE: If the table is overridden, then final_table will contain a
	 * mapped pointer to the full new table. If the table is not overridden,
	 * or if there has been a physical override, then the table will be
	 * fully mapped later (in verify table). In any case, we must
	 * unmap the header that was mapped above.
	 */
	final_table = acpi_tb_table_override(table, table_desc);
	if (!final_table) {
		final_table = table;	/* There was no override */
	}

	acpi_tb_print_table_header(table_desc->address, final_table);

	/* Set the global integer width (based upon revision of the DSDT) */

	if (table_index == ACPI_TABLE_INDEX_DSDT) {
		acpi_ut_set_integer_width(final_table->revision);
	}

	/*
	 * If we have a physical override during this early loading of the ACPI
	 * tables, unmap the table for now. It will be mapped again later when
	 * it is actually used. This supports very early loading of ACPI tables,
	 * before virtual memory is fully initialized and running within the
	 * host OS. Note: A logical override has the ACPI_TABLE_ORIGIN_OVERRIDE
	 * flag set and will not be deleted below.
	 */
	if (final_table != table) {
		acpi_tb_delete_table(table_desc);
	}

      unmap_and_exit:

	/* Always unmap the table header that we mapped above */

	acpi_os_unmap_memory(table, sizeof(struct acpi_table_header));
}

/*******************************************************************************
 *
>>>>>>> p9x
 * FUNCTION:    acpi_tb_get_root_table_entry
 *
 * PARAMETERS:  table_entry         - Pointer to the RSDT/XSDT table entry
 *              table_entry_size    - sizeof 32 or 64 (RSDT or XSDT)
 *
 * RETURN:      Physical address extracted from the root table
 *
 * DESCRIPTION: Get one root table entry. Handles 32-bit and 64-bit cases on
 *              both 32-bit and 64-bit platforms
 *
 * NOTE:        acpi_physical_address is 32-bit on 32-bit platforms, 64-bit on
 *              64-bit platforms.
 *
 ******************************************************************************/

static acpi_physical_address
acpi_tb_get_root_table_entry(u8 *table_entry, u32 table_entry_size)
{
	u64 address64;

	/*
	 * Get the table physical address (32-bit for RSDT, 64-bit for XSDT):
	 * Note: Addresses are 32-bit aligned (not 64) in both RSDT and XSDT
	 */
	if (table_entry_size == ACPI_RSDT_ENTRY_SIZE) {
		/*
		 * 32-bit platform, RSDT: Return 32-bit table entry
		 * 64-bit platform, RSDT: Expand 32-bit to 64-bit and return
		 */
		return ((acpi_physical_address)
			(*ACPI_CAST_PTR(u32, table_entry)));
	} else {
		/*
		 * 32-bit platform, XSDT: Truncate 64-bit to 32-bit and return
		 * 64-bit platform, XSDT: Move (unaligned) 64-bit to local,
		 *  return 64-bit
		 */
		ACPI_MOVE_64_TO_64(&address64, table_entry);

#if ACPI_MACHINE_WIDTH == 32
		if (address64 > ACPI_UINT32_MAX) {

			/* Will truncate 64-bit address to 32 bits, issue warning */

			ACPI_BIOS_WARNING((AE_INFO,
					   "64-bit Physical Address in XSDT is too large (0x%8.8X%8.8X),"
					   " truncating",
					   ACPI_FORMAT_UINT64(address64)));
		}
#endif
		return ((acpi_physical_address) (address64));
	}
}

/*******************************************************************************
 *
 * FUNCTION:    acpi_tb_parse_root_table
 *
 * PARAMETERS:  rsdp                    - Pointer to the RSDP
 *
 * RETURN:      Status
 *
 * DESCRIPTION: This function is called to parse the Root System Description
 *              Table (RSDT or XSDT)
 *
 * NOTE:        Tables are mapped (not copied) for efficiency. The FACS must
 *              be mapped and cannot be copied because it contains the actual
 *              memory location of the ACPI Global Lock.
 *
 ******************************************************************************/

acpi_status __init acpi_tb_parse_root_table(acpi_physical_address rsdp_address)
{
	struct acpi_table_rsdp *rsdp;
	u32 table_entry_size;
	u32 i;
	u32 table_count;
	struct acpi_table_header *table;
	acpi_physical_address address;
	u32 length;
	u8 *table_entry;
	acpi_status status;
	u32 table_index;

	ACPI_FUNCTION_TRACE(tb_parse_root_table);

	/* Map the entire RSDP and extract the address of the RSDT or XSDT */

	rsdp = acpi_os_map_memory(rsdp_address, sizeof(struct acpi_table_rsdp));
	if (!rsdp) {
		return_ACPI_STATUS(AE_NO_MEMORY);
	}

	acpi_tb_print_table_header(rsdp_address,
				   ACPI_CAST_PTR(struct acpi_table_header,
						 rsdp));

	/* Use XSDT if present and not overridden. Otherwise, use RSDT */

	if ((rsdp->revision > 1) &&
	    rsdp->xsdt_physical_address && !acpi_gbl_do_not_use_xsdt) {
		/*
		 * RSDP contains an XSDT (64-bit physical addresses). We must use
		 * the XSDT if the revision is > 1 and the XSDT pointer is present,
		 * as per the ACPI specification.
		 */
		address = (acpi_physical_address) rsdp->xsdt_physical_address;
		table_entry_size = ACPI_XSDT_ENTRY_SIZE;
	} else {
		/* Root table is an RSDT (32-bit physical addresses) */

		address = (acpi_physical_address) rsdp->rsdt_physical_address;
		table_entry_size = ACPI_RSDT_ENTRY_SIZE;
	}

	/*
	 * It is not possible to map more than one entry in some environments,
	 * so unmap the RSDP here before mapping other tables
	 */
	acpi_os_unmap_memory(rsdp, sizeof(struct acpi_table_rsdp));

	/* Map the RSDT/XSDT table header to get the full table length */

	table = acpi_os_map_memory(address, sizeof(struct acpi_table_header));
	if (!table) {
		return_ACPI_STATUS(AE_NO_MEMORY);
	}

	acpi_tb_print_table_header(address, table);

	/*
	 * Validate length of the table, and map entire table.
	 * Minimum length table must contain at least one entry.
	 */
	length = table->length;
	acpi_os_unmap_memory(table, sizeof(struct acpi_table_header));

	if (length < (sizeof(struct acpi_table_header) + table_entry_size)) {
		ACPI_BIOS_ERROR((AE_INFO,
				 "Invalid table length 0x%X in RSDT/XSDT",
				 length));
		return_ACPI_STATUS(AE_INVALID_TABLE_LENGTH);
	}

	table = acpi_os_map_memory(address, length);
	if (!table) {
		return_ACPI_STATUS(AE_NO_MEMORY);
	}

	/* Validate the root table checksum */

	status = acpi_tb_verify_checksum(table, length);
	if (ACPI_FAILURE(status)) {
		acpi_os_unmap_memory(table, length);
		return_ACPI_STATUS(status);
	}

	/* Get the number of entries and pointer to first entry */

	table_count = (u32)((table->length - sizeof(struct acpi_table_header)) /
			    table_entry_size);
	table_entry = ACPI_ADD_PTR(u8, table, sizeof(struct acpi_table_header));

	/*
	 * First two entries in the table array are reserved for the DSDT
	 * and FACS, which are not actually present in the RSDT/XSDT - they
	 * come from the FADT
	 */
	acpi_gbl_root_table_list.current_table_count = 2;

	/* Initialize the root table array from the RSDT/XSDT */

	for (i = 0; i < table_count; i++) {

		/* Get the table physical address (32-bit for RSDT, 64-bit for XSDT) */

		address =
		    acpi_tb_get_root_table_entry(table_entry, table_entry_size);

		/* Skip NULL entries in RSDT/XSDT */

		if (!address) {
			goto next_table;
		}

		status = acpi_tb_install_standard_table(address,
							ACPI_TABLE_ORIGIN_INTERNAL_PHYSICAL,
							FALSE, TRUE,
							&table_index);

		if (ACPI_SUCCESS(status) &&
		    ACPI_COMPARE_NAME(&acpi_gbl_root_table_list.
				      tables[table_index].signature,
				      ACPI_SIG_FADT)) {
			acpi_tb_parse_fadt(table_index);
		}

next_table:

		table_entry += table_entry_size;
	}

	acpi_os_unmap_memory(table, length);

	return_ACPI_STATUS(AE_OK);
}
