SECTION_DATA_PROLOGUE(.wlan_data, (NOLOAD),)
{
    *(.wlan_data)
    *(.wlan_data.*)
} GROUP_NOLOAD_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)
