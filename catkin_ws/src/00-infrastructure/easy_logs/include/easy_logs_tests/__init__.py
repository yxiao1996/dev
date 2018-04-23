

def jobs_comptests(context):  
    from . import summary 
    from . import slicing

    from comptests.registrar import jobs_registrar_simple
    jobs_registrar_simple(context)
    

