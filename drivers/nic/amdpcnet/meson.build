inc = [ 'include' ]

nic_amdpcnet_lib = static_library('nic-amdpcnet', 'src/amdpcnet.cpp',
	include_directories : [ '../../../servers/netserver/include', inc ],
	dependencies : deps,
	install : true
)

nic_amdpcnet_dep = declare_dependency(
	include_directories : inc,
	dependencies : deps,
	link_with : nic_amdpcnet_lib
)
