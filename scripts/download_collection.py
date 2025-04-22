# Usage
#     python3 download_collection.py -o <collection_owner> -c <collection_name>
#
# Description
#     This script will download all models contained within a collection.
#
import sys,json,requests
import getopt

if sys.version_info[0] < 3:
    raise Exception("Python 3 or greater is required. Try running `python3 download_collection.py`")

collection_name = ''
owner_name = ''

# Read options
optlist, args = getopt.getopt(sys.argv[1:], 'o:c:')

sensor_config_file = ''
private_token = ''
for o, v in optlist:
    if o == "-o":
        owner_name = v.replace(" ", "%20")
    if o == "-c":
        collection_name = v.replace(" ", "%20")

if not owner_name:
    print('Error: missing `-o <owner_name>` option')
    quit()

if not collection_name:
    print('Error: missing `-c <collection_name>` option')
    quit()


print("Downloading models from the {}/{} collection.".format(owner_name, collection_name.replace("%20", " ")))

page = 1
count = 0

# The Fuel server URL.
base_url ='https://fuel.gazebosim.org/'

# Fuel server version.
fuel_version = '1.0'

# Path to get the models in the collection
next_url = '/models?page={}&per_page=100&q=collections:{}'.format(page,collection_name)

# Path to download a single model in the collection
download_url = base_url + fuel_version + '/{}/models/'.format(owner_name)

# Iterate over the pages
while True:
    url = base_url + fuel_version + next_url

    # Get the contents of the current page.
    r = requests.get(url)

    if not r or not r.text:
        break

    # Convert to JSON
    models = json.loads(r.text)

    # Compute the next page's URL
    page = page + 1
    next_url = '/models?page={}&per_page=100&q=collections:{}'.format(page,collection_name)
  
    # Download each model 
    for model in models:
        count+=1
        model_name = model['name']
        print ('Downloading (%d) %s' % (count, model_name))
        download = requests.get(download_url+model_name+'.zip', stream=True)
        with open(model_name+'.zip', 'wb') as fd:
            for chunk in download.iter_content(chunk_size=1024*1024):
                fd.write(chunk)

print('Done.')